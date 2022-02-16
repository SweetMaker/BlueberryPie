#include <dummy.h>
#include <MotionSensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <EEPROM.h>
#include <SweetMaker.h>
#include <StrawberryString.h>
#include "BlueberryPie.h"

using namespace SweetMaker;

BlueberryPie myPie;

void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);


void setup()
{
	int retVal;
	Serial.begin(112500);
	Serial.println("MadCap");

	myPie.configEventHandlerCallback(myEventHandler);
	retVal = myPie.init();
	if (retVal != 0)
	{
		Serial.println("myPie init failure");
	}

}


void loop()
{
	myPie.update();
}


void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo)
{
	switch (eventId)
	{
	case TimerTickMngt::TIMER_TICK_100MS:
	case TimerTickMngt::TIMER_TICK_UPDATE:
	case TimerTickMngt::TIMER_TICK_10S:
		break;

	case TimerTickMngt::TIMER_TICK_S:

		if (Serial.available())
		{
			char c = Serial.read();
			Serial.println(c);

			switch (c)
			{

			case 'e':
			{
				if (!EEPROM.begin(0x400))
				{
					Serial.println("begin error");
				}
				uint8_t readU8 = EEPROM.readByte(0);
				Serial.print("I read: ");
				Serial.println(readU8);
				Serial.println("Writing '3'");
				EEPROM.writeByte(0, 3);
				if (!EEPROM.commit())
					Serial.println("Commit error");
				readU8 = EEPROM.readByte(0);
				Serial.print("I read: ");
				Serial.println(readU8);
			}
			break;

			case 'c':
			{
				myPie.midiBle.setMidiMsg(60);
			}
			break;

			case 'd':
			{
				myPie.midiBle.setMidiMsg(62);
			}
			break;
			case 'f':
			{
				myPie.midiBle.setMidiMsg(64);
			}
			break;
			}
		}
		break;

	case MotionSensor::MOTION_SENSOR_INIT_ERROR:
		Serial.println("MotionSensor Init Error");
		break;

	case MotionSensor::MOTION_SENSOR_READY:
		Serial.println("MotionSensor Ready");
		break;

	case MotionSensor::MOTION_SENSOR_RUNTIME_ERROR:
		Serial.println("MotionSensor Run Time Error");
		break;

	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:	
		break;

	case MidiBle::BME_CONNECT:
		Serial.println("BLE Connected");
		break;

	case MidiBle::BME_DISCONNECT:
		Serial.println("BLE Disconnected");
		break;


	default:
		Serial.print("Event: ");
		Serial.println(eventId);
			break;
	}
}


