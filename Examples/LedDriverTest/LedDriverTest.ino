#include <Arduino.h>
#include <EEPROM.h>
#include <Colour.h>
#include <SweetMaker.h>
#include "LedStripDriver.h"

using namespace SweetMaker;

ColourRGB rgbString[5];

ILedStripDriver * myLedDriver;

SigGen rgbSig[3];

void myEventHandler(uint16_t eventType, uint8_t srcId, uint16_t eventInfo);

void setup()
{
	Serial.begin(112500);
	Serial.println("Led Driver");

	EventMngr::getMngr()->configCallBack(myEventHandler);


	myLedDriver = new Esp32LedStripDriver(27, 5);



	for (int i = 0; i < 5; i++) {
		rgbString[i].blue = 14;
		rgbString[i].green = 56;
		rgbString[i].red = 92;
	}

	rgbSig[0].configSamples(sineWave255, NUM_SAM(sineWave255), 3000, 0);
	rgbSig[1].configSamples(sineWave255, NUM_SAM(sineWave255), 6000, 0);
	rgbSig[2].configSamples(sineWave255, NUM_SAM(sineWave255), 9000, 0);
	
	rgbSig[0].start();
	rgbSig[1].start();
	rgbSig[2].start();

}

void loop()
{
	PerfMon::getPerfMon()->intervalStop();
	PerfMon::getPerfMon()->intervalStart();

	for (int i = 0; i < 5; i++) {
		rgbString[i].blue = rgbSig[0].readValue();
		rgbString[i].green = rgbSig[1].readValue();
		rgbString[i].red = rgbSig[2].readValue();
	}

	myLedDriver->show(rgbString, 5);

	AutoUpdateMngr::getUpdater()->update();
	TimerTickMngt::getTimerMngt()->update(0);
}


void myEventHandler(uint16_t eventType, uint8_t srcId, uint16_t eventInfo)
{
	if (eventType == TimerTickMngt::TIMER_TICK_10S)
	{
	//	PerfMon::getPerfMon()->print();
		PerfMon::getPerfMon()->reset();
	}
}
