/*
 * Distiller scale
 *
 * Copyright (C) 2017 Edwin Croissant
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * See the README.md file for additional information.
 */

#include "Arduino.h"
#include <EEPROM.h>
#include <math.h>
#include <OneWire.h> // https://github.com/bigjosh/OneWireNoResistor
#include <SingleDS18B20.h>	// https://github.com/EdwinCroissantArduinoLibraries/SingleDS18B20
#include <SimpleKeyHandler.h> // https://github.com/EdwinCroissantArduinoLibraries/SimpleKeyHandler
#include <SimpleHX711.h> // https://github.com/EdwinCroissantArduinoLibraries/SimpleHX711
#include <SSD1306Ascii.h> // https://github.com/EdwinCroissantArduinoLibraries/SSD1306Ascii
#include <SSD1306AsciiAvrI2c.h>

#define VERSION "00.01"
/* version history:
 * 00.01 13may2017 First release
 */

// recognizable names for the pins

enum pins {
	pinDS18B20Case = A1,
	pinDS18B20Beam = A2,
	pinHX711Clk = 12,
	pinHX711Data = 11,
	pinBeeper = A0,
	pinLeftBtn = 5,
	pinRightBtn = 4,
	pinBlinker = 13
};

// recognizable names for the EEPROM addresses

enum EEPROMAdr {
	eeIndentifier = 0,				// 2 bytes for identifier use
	eeAdjuster = 2,					// 4 bytes for adjuster
	eeTare = 6,						// 4 bytes for tare
	eeTareTemp = 10,				// 2 bytes for temp
	eeSpare = 12, 					// 4 bytes for spare
	eeTemperatureCoefficient = 16,	// 2 bytes for temperature coefficient
	eeCalWeight = 18,				// 2 bytes for the calibration weight
	eeAlarm = 20,					// 8 bytes for the alarm values
	eeLastEntry = 28
};

// setup oneWire instances to communicate with the Dallas Sensors

OneWire oneWirePinCase(pinDS18B20Case);
OneWire oneWirePinBeam(pinDS18B20Beam);

// setup SingleDS18B20 instances and pass our oneWire references.

SingleDS18B20 caseSensor(&oneWirePinCase);
SingleDS18B20 beamSensor(&oneWirePinBeam);

// setup a SingleHX711 instance and pass our pins.

SimpleHX711 scale(pinHX711Clk, pinHX711Data);

// setup an instance of the SSD1306 Oled display

SSD1306AsciiAvrI2c oled;
// Setup instances for the keys

SimpleKeyHandler rightBtn;
SimpleKeyHandler leftBtn;

// declare variables

enum historySize {
	buffersize = 64, buffermask = 64 - 1
};
// defines the size of the history buffer

struct TimestampedWeight {
	int32_t weight10X;
	uint32_t timestamp;
};

struct data {
	TimestampedWeight history[buffersize];
	uint8_t historyNewest;
	int16_t calWeight;
	int16_t temperatureCoefficient;  // mg/C
	int16_t measuredTC; // mg/C
	int16_t tareTemp;
	int16_t tempComp; // in deci gram
	uint32_t timestamp;
	int16_t weight; // in gram
	int32_t weight10X; // in deci gram
	int32_t rate; // in deci gram per minute
	uint32_t remainingTime; // in seconds
	bool alarmOn;
	uint32_t alarmTimeOn;
	int32_t weightAtTimeOn; // in deci gram
	uint32_t alarmTimeOff;
	int32_t weightAtTimeOff; // in deci gram
	uint8_t alarmSelected;
	int16_t alarm[4];
	uint8_t measuretime;
	uint8_t settingSelected;
} Data;

enum EditSelection {
	sign = -1,
	d0 = 0,
	d1 = 1,
	d2 = 2,
	d3 = 3,
	extra = 4,
	save = 5,
	cancel = 6
};

struct editData {
	bool isMinus;bool useMinus;
	uint8_t digit[4];
	EditSelection selected;
	void (*onCancel)();
	void (*onSave)();
	void (*onExtra)();
	void (*onRefreshPage)();
} EditData;

uint32_t Last1000Update, Last250Update;
char scratchpad[21];
void (*AutoPageRefreshFast)();
void (*AutoPageRefreshSlow)();
void (*ReturnPage)();

// Helper function to call a function at a certain interval

void doFunctionAtInterval(void (*callBackFunction)(), uint32_t *lastEvent,
		uint32_t Interval) {
	uint32_t now = millis();
	if ((now - *lastEvent) >= Interval) {
		*lastEvent = now;
		callBackFunction();
	}
}

//the setup function is called once at startup of the sketch
void setup() {
	// setup the pins
	pinMode(pinLeftBtn, INPUT_PULLUP);
	pinMode(pinRightBtn, INPUT_PULLUP);
	pinMode(pinBeeper, OUTPUT);
	digitalWrite(pinBeeper, LOW);
	pinMode(pinBlinker, OUTPUT);
	digitalWrite(pinBlinker, LOW);
	// couple the two keys
	leftBtn.setCompanion(&rightBtn);

	// setup the oled display
	oled.begin(&Adafruit128x64, 0x3C);
	oled.clear();

	// show the splash screen
	pageSplashInit();

	//initialize the temperature sensors
	caseSensor.setResolution(SingleDS18B20::res12bit);
	caseSensor.convert();
	beamSensor.setResolution(SingleDS18B20::res12bit);
	beamSensor.convert();

	Last1000Update = millis();

	// read EEPROM
	if (!loadFromEEPROM())
		clearEEPROM();

	/*
	 * start with a blocking call to read the scale and the DS18B20,
	 * at 10Hz the initialization time of the scale is about
	 * 400 ms, the DS180B20 takes 750 ms
	 */
	while (!scale.read() || millis() < (Last1000Update + 1000))
		;

	// show the weight screen
	pageWeightInit();
}

// The loop function is called in an endless loop
void loop() {
	scale.read();
	doFunctionAtInterval(update250ms, &Last250Update, 250);
	doFunctionAtInterval(update1000ms, &Last1000Update, 1000);

	// handle the buttons
	leftBtn.read(!digitalRead(pinLeftBtn));
	rightBtn.read(!digitalRead(pinRightBtn));

	// handle the alarm
	digitalWrite(pinBeeper,
			Data.alarmOn && (Data.weight >= Data.alarm[Data.alarmSelected]));
}

void update250ms() {
	Data.timestamp = scale.getTimestamp();
	Data.weight10X = scale.getAdjusted(false) + Data.tempComp;
	Data.weight = divNearest(Data.weight10X, 10);

	if (AutoPageRefreshFast)
		AutoPageRefreshFast();
}

void update1000ms() {
	// update weight history
	Data.historyNewest++;
	Data.historyNewest &= (buffermask);
	Data.history[Data.historyNewest].timestamp = Data.timestamp;
	Data.history[Data.historyNewest].weight10X = Data.weight10X;

	// update temperature
	caseSensor.read();
	caseSensor.convert();
	beamSensor.read();
	beamSensor.convert();

	// calculate the temperature compensation
	Data.tempComp = divNearest(
			int32_t(Data.temperatureCoefficient)
					* int32_t(Data.tareTemp - beamSensor.getTempAsRaw()),
			1600L);

	// calculate remaining time
	if (Data.alarmOn && (Data.weight10X - Data.weightAtTimeOn)) {
		uint32_t RunningTime = scale.getTimestamp() - Data.alarmTimeOn;
		int32_t RunningWeight = Data.weight10X - Data.weightAtTimeOn;
		if (RunningWeight < 0)
			goto error;
		int32_t RemainingWeight = Data.alarm[Data.alarmSelected] * 10
				- Data.weight10X;
		if (RemainingWeight < 0)
			goto error;

		Data.remainingTime = int32_t(RunningTime) / RunningWeight
				* RemainingWeight;
		Data.remainingTime /= 1000UL; // from millis to seconds
		if (Data.remainingTime > 15359UL) // more then 255 minutes 59 seconds
			goto error;
	} else
		error: Data.remainingTime = 0;

	// calculate the rate
	// TODO need check for data consistency.

	uint8_t oldIndex;

	if (!Data.measuretime) {
		if (Data.alarmOn) {
			Data.rate = calculateRate(Data.weightAtTimeOn, Data.weight10X,
					Data.alarmTimeOn, Data.timestamp);
		} else {
			Data.rate = calculateRate(Data.weightAtTimeOn, Data.weightAtTimeOff,
					Data.alarmTimeOn, Data.alarmTimeOff);
		}
	} else {
		oldIndex = (Data.historyNewest - Data.measuretime) & (buffermask);
		Data.rate = calculateRate(Data.history[oldIndex].weight10X,
				Data.weight10X, Data.history[oldIndex].timestamp,
				Data.timestamp);
	}

	if (AutoPageRefreshSlow)
		AutoPageRefreshSlow();

	// blink led;
	digitalWrite(pinBlinker, !digitalRead(pinBlinker));
}

// helper function to calculate the rate
int32_t calculateRate(int32_t oldWeight, int32_t newWeight, uint32_t oldTime,
		uint32_t newTime) {
	return oldTime == newTime ?
			0 : (newWeight - oldWeight) * 60000L
/ int32_t(newTime - oldTime);
}

void pageSplashInit() {
	oled.setFont(Arial14);
	oled.setCursor(18, 0);
	oled.print(F("Edwin Croissant"));
	oled.setCursor(26, 3);
	oled.print(F("Distiller scale"));
	oled.setCursor(8, 6);
	oled.print(F("www.visionstills.org"));
}

void pageWeightInit() {
	leftBtn.clear();
	leftBtn.onShortPress = toggleAlarm;
	leftBtn.onLongPress = pageAlarmsInit;
	leftBtn.onBothPress = pageSettingsInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageRemainingInit;
	rightBtn.onLongPress = tare;
	AutoPageRefreshFast = pageWeightRefreshFast;
	AutoPageRefreshSlow = pageWeightRefreshSlow;
	ReturnPage = pageWeightInit;
	oled.clear();
	AutoPageRefreshFast();
	AutoPageRefreshSlow();
}

void pageWeightRefreshFast() {
	(Data.weight10X < -9999) || (Data.weight10X > 99999) ?
			strcpy(scratchpad, "----.-") :
			dtostrf(float(Data.weight10X) / 10, 6, 1, scratchpad);
	oled.setFont(lcdnums14x24);
	oled.set2X();
	oled.setCursor(0, 2);
	oled.print(char(scratchpad[0]));
	oled.print(char(scratchpad[1]));
	oled.print(char(scratchpad[2]));
	oled.print(char(scratchpad[3]));
	oled.set1X();
	oled.setCursor(114, 2);
	oled.print(char(scratchpad[5]));
}

void pageWeightRefreshSlow() {
	oled.setFont(X11fixed7x14);
	oled.setCursor(0, 0);
	oled.setInverted(Data.alarmOn);
	oled.print(F("Alarm "));
	dtostrf(Data.alarm[Data.alarmSelected], 4, 0, scratchpad);
	oled.print(scratchpad);
	oled.print('g');
	oled.setInverted(false);
	oled.setCursor(86, 0);
	uint8_t minutes, seconds;
	if (Data.remainingTime) {
		minutes = (uint8_t) (Data.remainingTime / 60);
		seconds = (uint8_t) (Data.remainingTime - minutes * 60);
		sprintf(scratchpad, "%3.1d:%.2d", minutes, seconds);
	} else
		strcpy(scratchpad, "  -:--");
	oled.print(scratchpad);
}


void pageRemainingInit() {
	leftBtn.clear();
	leftBtn.onShortPress = toggleAlarm;
	leftBtn.onLongPress = pageAlarmsInit;
	leftBtn.onBothPress = pageSettingsInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageRateInit;
	rightBtn.onLongPress = tare;
	AutoPageRefreshFast = pageRemainingRefreshFast;
	AutoPageRefreshSlow = pageRemainingRefreshSlow;
	ReturnPage = pageRemainingInit;
	oled.clear();
	AutoPageRefreshFast();
	AutoPageRefreshSlow();
}

void pageRemainingRefreshFast() {
	oled.setFont(X11fixed7x14);
	oled.setCursor(79, 0);
	dtostrf(Data.weight, 5, 0, scratchpad);
	oled.print(scratchpad);
	oled.print(F(" g"));
}

void pageRemainingRefreshSlow() {
	oled.setFont(X11fixed7x14);
	oled.setCursor(0, 0);
	oled.setInverted(Data.alarmOn);
	oled.print(F("Alarm "));
	dtostrf(Data.alarm[Data.alarmSelected], 4, 0, scratchpad);
	oled.print(scratchpad);
	oled.print('g');
	oled.setInverted(false);
	uint8_t minutes, seconds;
	if (Data.remainingTime) {
		minutes = (uint8_t) (Data.remainingTime / 60);
		seconds = (uint8_t) (Data.remainingTime - minutes * 60);
		sprintf(scratchpad, "%3.1d%.2d", minutes, seconds);
	} else
		strcpy(scratchpad, "  ---");
	oled.setFont(lcdnums14x24);
	oled.set2X();
	oled.setCursor(14, 2);
	oled.print(char(scratchpad[0]));
	oled.print(char(scratchpad[1]));
	oled.print(char(scratchpad[2]));
	oled.set1X();
	oled.setCursor(100, 2);
	oled.print(char(scratchpad[3]));
	oled.print(char(scratchpad[4]));
}

void pageRateInit() {
	leftBtn.clear();
	leftBtn.onShortPress = increaseMeasuretime;
	leftBtn.onRepPress = increaseMeasuretime;
	leftBtn.onBothPress = pageSettingsInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageWeightInit;
	rightBtn.onLongPress = tare;
	AutoPageRefreshFast = pageRateRefreshFast;
	AutoPageRefreshSlow = pageRateRefreshSlow;
	ReturnPage = pageRateInit;
	oled.clear();
	AutoPageRefreshFast();
	AutoPageRefreshSlow();
}

void pageRateRefreshFast() {
	oled.setFont(X11fixed7x14);
	oled.setCursor(0, 0);
	oled.print(F("dt"));
	if (Data.measuretime) {
		dtostrf(Data.measuretime, 3, 0, scratchpad);
		oled.print(scratchpad);
		oled.print(F(" s"));
	} else
		oled.print(F("  run"));
	oled.setCursor(79, 0);
	dtostrf(Data.weight, 5, 0, scratchpad);
	oled.print(scratchpad);
	oled.print(F(" g"));
}

void pageRateRefreshSlow() {
	(Data.rate < -9999) || (Data.rate > 99999) ?
			strcpy(scratchpad, "----.-") :
			dtostrf(float(Data.rate) / 10, 6, 1, scratchpad);
	oled.set2X();
	oled.setFont(lcdnums14x24);
	oled.setCursor(0, 2);
	oled.print(char(scratchpad[0]));
	oled.print(char(scratchpad[1]));
	oled.print(char(scratchpad[2]));
	oled.print(char(scratchpad[3]));
	oled.set1X();
	oled.setCursor(114, 2);
	oled.print(char(scratchpad[5]));
}


void pageSettingsInit() {
	AutoPageRefreshFast = nullptr;
	AutoPageRefreshSlow = nullptr;
	// assign functions to the key handlers
	leftBtn.clear();
	leftBtn.onShortPress = pageNextSetting;
	leftBtn.onLongPress = ReturnPage;
	rightBtn.clear();
	oled.clear();
	oled.setFont(X11fixed7x14);
	pageSettingsRefresh();
}

void pageSettingsRefresh() {
	oled.setCursor(0, 0);
	if (Data.settingSelected == 0) {
		rightBtn.onShortPress = pageDataInit;
		oled.setInverted(true);
	} else
		oled.setInverted(false);
	oled.print(F("Show data"));
	oled.clearToEOL();

	oled.setCursor(0, 2);
	if (Data.settingSelected == 1) {
		rightBtn.onShortPress = saveTare;
		oled.setInverted(true);
	} else
		oled.setInverted(false);
	oled.print(F("Save tare"));
	oled.clearToEOL();

	oled.setCursor(0, 4);
	if (Data.settingSelected == 2) {
		rightBtn.onShortPress = pageEditTCInit;
		oled.setInverted(true);
	} else
		oled.setInverted(false);
	oled.print(F("Set temp. comp."));
	oled.clearToEOL();

	oled.setCursor(0, 6);
	if (Data.settingSelected == 3) {
		rightBtn.onShortPress = pageCalibrateInit;
		oled.setInverted(true);
	} else
		oled.setInverted(false);
	oled.print(F("Calibrate"));
	oled.clearToEOL();
	oled.setInverted(false);

//	dtostrf(Data.calWeight, 4, 0, scratchpad);
//	oled.print(scratchpad);
}

void pageNextSetting() {
	Data.settingSelected++;
	Data.settingSelected &= B00000011;
	pageSettingsRefresh();

}

void numberToDigitArray(const int16_t n) {
	int16_t temp;
	EditData.isMinus = n < 0;
	temp = abs(n);
	int i;
	for (i = 3; i >= 0; i--) {
		EditData.digit[i] = temp % 10;
		temp /= 10;
	}
}

int16_t digitArraytoNumber() {
	int i, k = 0;
	for (i = 0; i < 4; i++)
		k = 10 * k + EditData.digit[i];
	if (EditData.isMinus)
		k *= -1;
	return k;
}

void pageAlarmsInit() {
	AutoPageRefreshFast = pageAlarmsRefreshWeight;
	AutoPageRefreshSlow = nullptr;
	// assign functions to the key handlers
	leftBtn.clear();
	leftBtn.onShortPress = pageNextAlarm;
	leftBtn.onLongPress = ReturnPage;
	rightBtn.clear();
	rightBtn.onShortPress = ReturnPage;
	rightBtn.onLongPress = pageEditAlarmInit;
	rightBtn.onBothPress = setAlarmByWeight;
	oled.clear();
	oled.setFont(X11fixed7x14);
	pageAlarmsRefresh();
}

void pageNextAlarm() {
	Data.alarmSelected++;
	Data.alarmSelected &= B00000011;
	// Data.alarmSelected > 2 ? Data.alarmSelected = 0 : Data.alarmSelected++;
	pageAlarmsRefresh();
}

void pageAlarmsRefresh() {
	for (int i = 0; i < 4; ++i) {
		oled.setCursor(0, i * 2);
		Data.alarmSelected == i ?
				oled.setInverted(true) : oled.setInverted(false);
		oled.print(F("Alarm "));
		dtostrf(Data.alarm[i], 4, 0, scratchpad);
		oled.print(scratchpad);
		Data.alarmSelected == i ? oled.clearToCol(79) : oled.clearToEOL();
	}
	oled.setInverted(false);
	pageAlarmsRefreshWeight();
}

void pageAlarmsRefreshWeight() {
	oled.setCursor(79, Data.alarmSelected * 2);
	dtostrf(Data.weight, 5, 0, scratchpad);
	oled.setInverted(true);
	oled.print(scratchpad);
	oled.print(F(" g"));
	oled.setInverted(false);
}

void pageEditAlarmInit() {
	AutoPageRefreshSlow = nullptr;
	AutoPageRefreshFast = nullptr;
	leftBtn.clear();
	leftBtn.onShortPress = nextDigit;
	leftBtn.onLongPress = pageAlarmsInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageAlarmsInit;
	numberToDigitArray(Data.alarm[Data.alarmSelected]);
	oled.clear();
	EditData.selected = cancel;
	EditData.onCancel = pageAlarmsInit;
	EditData.onSave = setAlarm;
	EditData.onExtra = nullptr;
	EditData.useMinus = false;
	EditData.onRefreshPage = pageEditRefresh;
	pageEditRefresh();
}

void pageEditRefresh() {
	oled.setFont(X11fixed7x14);
	EditData.selected == cancel ?
			oled.setInverted(true) : oled.setInverted(false);
	oled.setCursor(0, 0);
	oled.print(F(" Cancel "));
	EditData.selected == save ?
			oled.setInverted(true) : oled.setInverted(false);
	oled.setCol(86);
	oled.print(F(" Save "));
	oled.setFont(lcdnums14x24);
	oled.set2X();
	oled.setCursor(5, 2);
	EditData.selected == d0 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[0], DEC);
	oled.setCol(35);
	EditData.selected == d1 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[1], DEC);
	oled.setCol(65);
	EditData.selected == d2 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[2], DEC);
	oled.setCol(95);
	EditData.selected == d3 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[3], DEC);
	oled.set1X();
	oled.setInverted(false);
}

void nextDigit() {
	EditData.selected < cancel ?
			EditData.selected = EditSelection(EditData.selected + 1) :
	EditData.useMinus ? EditData.selected = sign : EditData.selected = d0;

	if (!EditData.onExtra && EditData.selected == extra)
		EditData.selected = EditSelection(EditData.selected + 1);

	switch (EditData.selected) {
	case cancel: {
		rightBtn.onShortPress = EditData.onCancel;
		rightBtn.onRepPress = nullptr;
		break;
	}
	case save: {
		rightBtn.onShortPress = EditData.onSave;
		rightBtn.onRepPress = nullptr;
		break;
	}
	case extra: {
		rightBtn.onShortPress = EditData.onExtra;
		rightBtn.onRepPress = nullptr;
		break;
	}
	case sign: {
		rightBtn.onShortPress = changeSign;
		rightBtn.onRepPress = nullptr;
		break;
	}
	default:
		rightBtn.onShortPress = increaseDigit;
		rightBtn.onRepPress = increaseDigit;
	}
	EditData.onRefreshPage();
}

void increaseDigit() {
	EditData.digit[EditData.selected] < 9 ?
			EditData.digit[EditData.selected]++ :
			EditData.digit[EditData.selected] = 0;
	EditData.onRefreshPage();
}

void changeSign() {
	EditData.isMinus = !EditData.isMinus;
	EditData.onRefreshPage();
}


void saveTare() {
	scale.tare(true);
	Data.tareTemp = beamSensor.getTempAsRaw();
	EEPROMupdate32(eeTare, scale.getTare());
	EEPROMupdate16(eeTareTemp, Data.tareTemp);
	ReturnPage();
}

void pageCalibrateInit() {
	leftBtn.clear();
	leftBtn.onShortPress = pageEditCalWeightInit;
	leftBtn.onLongPress = pageSettingsInit;
	rightBtn.clear();
	rightBtn.onShortPress = adjustScale;
	rightBtn.onLongPress = tare;
	oled.clear();
	AutoPageRefreshFast = pageWeightRefreshFast;
	AutoPageRefreshSlow = nullptr;
	ReturnPage = pageWeightInit;
	oled.clear();
	oled.setFont(X11fixed7x14);
	oled.setCursor(0, 0);
	oled.print(F("Set to "));
	dtostrf(Data.calWeight, 4, 0, scratchpad);
	oled.print(scratchpad);
	oled.print('g');
	AutoPageRefreshFast();
}

void pageEditCalWeightInit() {
	AutoPageRefreshSlow = nullptr;
	AutoPageRefreshFast = nullptr;
	leftBtn.clear();
	leftBtn.onShortPress = nextDigit;
	leftBtn.onLongPress = pageCalibrateInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageCalibrateInit;
	numberToDigitArray(Data.calWeight);
	oled.clear();
	EditData.selected = cancel;
	EditData.onCancel = pageCalibrateInit;
	EditData.onSave = setCalWeight;
	EditData.onExtra = nullptr;
	EditData.useMinus = false;
	EditData.onRefreshPage = pageEditRefresh;
	pageEditRefresh();

}

void setCalWeight() {
	Data.calWeight = digitArraytoNumber();
	EEPROMupdate16(eeCalWeight, Data.calWeight);
	pageCalibrateInit();
}

void adjustScale() {
	scale.adjustTo(Data.calWeight * 10, true);
	EEPROMupdate32(eeAdjuster, scale.getAdjuster());
}

void pageDataInit() {
	// assign functions to the key handlers
	leftBtn.clear();
	leftBtn.onShortPress = ReturnPage;
	leftBtn.onLongPress = pageSettingsInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageScaleDataInit;

	AutoPageRefreshFast = pageDataRefresh;
	AutoPageRefreshSlow = nullptr;
	oled.clear();
	oled.setFont(Adafruit5x7);
	oled.setCursor(0, 0);
	oled.print(F("Case temp"));
	oled.setCursor(0, 1);
	oled.print(F("Case raw"));
	oled.setCursor(0, 2);
	oled.print(F("Beam temp"));
	oled.setCursor(0, 3);
	oled.print(F("Beam raw"));
	oled.setCursor(0, 4);
	oled.print(F("Tare temp"));
	oled.setCursor(0, 5);
	oled.print(F("Tare raw"));
	oled.setCursor(0, 7);
	oled.print(F("Version"));
}

void pageDataRefresh() {

	oled.setCursor(62, 0);
	dtostrf(caseSensor.getTempAsC(), 11, 1, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 1);
	dtostrf(caseSensor.getTempAsRaw(), 11, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 2);
	dtostrf(beamSensor.getTempAsC(), 11, 1, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 3);
	dtostrf(beamSensor.getTempAsRaw(), 11, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 4);
	dtostrf(float(Data.tareTemp) / 16, 11, 1, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 5);
	dtostrf(Data.tareTemp, 11, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(98, 7);
	oled.print(F(VERSION));
}

void pageScaleDataInit() {
	// assign functions to the key handlers
	leftBtn.clear();
	leftBtn.onShortPress = ReturnPage;
	leftBtn.onLongPress = pageSettingsInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageDataInit;
	rightBtn.onLongPress = tare;

	AutoPageRefreshFast = pageScaleDataRefresh;
	AutoPageRefreshSlow = nullptr;
	oled.clear();
	oled.setFont(Adafruit5x7);
	oled.setCursor(0, 0);
	oled.print(F("Raw"));
	oled.setCursor(0, 1);
	oled.print(F("Tare"));
	oled.setCursor(0, 2);
	oled.print(F("Raw - tare"));
	oled.setCursor(0, 3);
	oled.print(F("Adjuster"));
	oled.setCursor(0, 4);
	oled.print(F("Adjusted"));
	oled.setCursor(0, 5);
	oled.print(F("Compensation"));
	oled.setCursor(0, 6);
	oled.print(F("Compensated"));
	oled.setCursor(0, 7);
	oled.print(F("Temp Coeff"));
}

void pageScaleDataRefresh() {

	oled.setCursor(62, 0);
	dtostrf(scale.getRaw(true), 11, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 1);
	dtostrf(scale.getTare(), 11, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 2);
	dtostrf(scale.getRawMinusTare(true), 11, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(62, 3);
	dtostrf(scale.getAdjuster(), 11, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(79, 4);
	dtostrf(scale.getAdjusted(true), 8, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(79, 5);
	dtostrf(Data.tempComp, 8, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(79, 6);
	dtostrf(Data.weight10X, 8, 0, scratchpad);
	oled.print(scratchpad);

	oled.setCursor(79, 7);
	dtostrf(Data.temperatureCoefficient, 8, 0, scratchpad);
	oled.print(scratchpad);
}

void pageEditTCInit() {
	// reload the saved tare and temperature from eeprom
	scale.setTare(EEPROMread32(eeTare));
	Data.tareTemp = EEPROMread16(eeTareTemp);
	// calculate the temperature coefficient
	int32_t deltaTemp = beamSensor.getTempAsRaw() - Data.tareTemp;
	int32_t deltaTare = scale.getAdjusted(true);
	if (deltaTemp) {
		Data.measuredTC = int32_t(deltaTare) * 1600L / deltaTemp;
		if (Data.measuredTC > 9999 || Data.measuredTC < -9999)
			Data.measuredTC = 0;
	} else
		Data.measuredTC = 0;
	AutoPageRefreshSlow = nullptr;
	AutoPageRefreshFast = nullptr;
	leftBtn.clear();
	leftBtn.onShortPress = nextDigit;
	leftBtn.onLongPress = pageSettingsInit;
	rightBtn.clear();
	rightBtn.onShortPress = pageSettingsInit;
	numberToDigitArray(Data.temperatureCoefficient);
	oled.clear();
	EditData.selected = cancel;
	EditData.onCancel = pageSettingsInit;
	EditData.onSave = setTC;
	EditData.onExtra = takeOverTC;
	EditData.useMinus = true;
	EditData.onRefreshPage = pageEditTCRefresh;
	pageEditTCRefresh();
}

void pageEditTCRefresh() {
	oled.setFont(X11fixed7x14);
	EditData.selected == cancel ?
			oled.setInverted(true) : oled.setInverted(false);
	oled.setCursor(0, 0);
	oled.print(F(" Cancel "));
	EditData.selected == save ?
			oled.setInverted(true) : oled.setInverted(false);
	oled.setCol(86);
	oled.print(F(" Save "));
	oled.setInverted(false);
	oled.setCursor(0, 2);
	oled.print(F("Measured"));
	EditData.selected == extra ?
			oled.setInverted(true) : oled.setInverted(false);
	oled.setCol(93);
	dtostrf(Data.measuredTC, 5, 0, scratchpad);
	oled.print(scratchpad);
	oled.setFont(lcdnums12x16);
	oled.set2X();
	oled.setCursor(0, 4);
	EditData.selected == sign ?
			oled.setInverted(true) : oled.setInverted(false);
	EditData.isMinus ? oled.print('-') : oled.print(' ');
	oled.setCol(26);
	EditData.selected == d0 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[0], DEC);
	oled.setCol(52);
	EditData.selected == d1 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[1], DEC);
	oled.setCol(78);
	EditData.selected == d2 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[2], DEC);
	oled.setCol(104);
	EditData.selected == d3 ? oled.setInverted(true) : oled.setInverted(false);
	oled.print(EditData.digit[3], DEC);
	oled.set1X();
	oled.setInverted(false);
}

void setTC() {
	Data.temperatureCoefficient = digitArraytoNumber();
	EEPROMupdate16(eeTemperatureCoefficient, Data.temperatureCoefficient);
	pageSettingsInit();
}

void takeOverTC() {
	Data.temperatureCoefficient = Data.measuredTC;
	numberToDigitArray(Data.temperatureCoefficient);
	pageEditTCRefresh();
}

void tare() {
	scale.tare(true);
	Data.tareTemp = beamSensor.getTempAsRaw();
	Data.tempComp = 0;
}

void toggleAlarm() {
	Data.alarmOn = !Data.alarmOn;
	if (Data.alarmOn) {
		Data.alarmTimeOn = Data.timestamp;
		Data.weightAtTimeOn = Data.weight10X;
	} else {
		Data.alarmTimeOff = Data.timestamp;
		Data.weightAtTimeOff = Data.weight10X;
	}
	AutoPageRefreshSlow();
}

/*
 * TODO is it possible to automate this?
 */
void increaseMeasuretime() {
	switch (Data.measuretime) {
	case 0:
		Data.measuretime = 1;
		break;
	case 1:
		Data.measuretime = 2;
		break;
	case 2:
		Data.measuretime = 4;
		break;
	case 4:
		Data.measuretime = 16;
		break;
	case 16:
		Data.measuretime = 32;
		break;
	case 32:
		Data.measuretime = 63;
		break;
	default:
		Data.measuretime = 0;
		break;
	}
}

void setAlarmByWeight() {
	Data.alarm[Data.alarmSelected] = Data.weight;
	EEPROMupdate16(eeAlarm + (Data.alarmSelected * 2),
			Data.alarm[Data.alarmSelected]);
	pageAlarmsRefresh();
}

void setAlarm() {
	Data.alarm[Data.alarmSelected] = digitArraytoNumber();
	EEPROMupdate16(eeAlarm + (Data.alarmSelected * 2),
			Data.alarm[Data.alarmSelected]);
	pageAlarmsInit();
}

bool loadFromEEPROM() {
	if (EEPROMread16(eeIndentifier) == 123) {
		scale.setAdjuster(EEPROMread32(eeAdjuster));
		scale.setTare(EEPROMread32(eeTare));
		Data.tareTemp = EEPROMread16(eeTareTemp);
		Data.temperatureCoefficient = EEPROMread16(eeTemperatureCoefficient);
		Data.calWeight = EEPROMread16(eeCalWeight);
		for (uint8_t i = 0; i < 4; ++i)
		{
			Data.alarm[i] = EEPROMread16(eeAlarm + (i * 2));
		}
		return true;
	} else
		return false;
}

void clearEEPROM() {
	EEPROMupdate16(eeIndentifier, 123);
	EEPROMupdate16(eeCalWeight, 1000);
	EEPROMupdate32(eeAdjuster, 256);
	for (int i = 8; i < eeLastEntry; ++i) {
		EEPROM.update(i, 0);
	}
}

// Helper functions to read and write the EEPROM

int16_t EEPROMread16(int address) {
	int16_t result;
	for (int i = 0; i < 2; ++i) {
		reinterpret_cast<uint8_t*>(&result)[i] = EEPROM.read(address + i);
	}
	return result;
}

void EEPROMupdate16(int address, int16_t value) {
	for (int i = 0; i < 2; ++i) {
		EEPROM.update(address + i, reinterpret_cast<uint8_t*>(&value)[i]);
	}
}

int32_t EEPROMread32(int address) {
	int32_t result;
	for (int i = 0; i < 4; ++i) {
		reinterpret_cast<uint8_t*>(&result)[i] = EEPROM.read(address + i);
	}
	return result;
}

void EEPROMupdate32(int address, int32_t value) {
	for (int i = 0; i < 4; ++i) {
		EEPROM.update(address + i, reinterpret_cast<uint8_t*>(&value)[i]);
	}
}

int32_t divNearest(int32_t dividend, int32_t divisor) {
	/*
	 *  TODO This has a potential overflow problem
	 */
	if (divisor < 0)
		dividend *= -1, divisor *= -1;
	return (abs(dividend) + ((divisor - (dividend < 0 ? 1 : 0)) >> 1)) / divisor
			* ((dividend < 0) ? -1 : +1);
}

