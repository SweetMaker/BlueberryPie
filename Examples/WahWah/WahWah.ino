/*
 * LOLIN D32 PRO
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
 */

#include <Wire.h>
#include "ToDiscrete.h"
#include "BlueberryPie.h"

using namespace SweetMaker;

BlueberryPie myPie;

void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);


void setup()
{
	int retVal;
	Serial.begin(112500);
	Serial.println("Welcome to Guitar Wahwah");

	myPie.configEventHandlerCallback(myEventHandler);
	retVal = myPie.init();
	if (retVal != 0)
	{
		Serial.println("myPie init failure");
	}
	Serial.println("myPie init complete");
}

void loop()
{
	myPie.update();
	handleSerialInput();
}

void handleSerialInput()
{
	if (Serial.available())
	{
		char c = Serial.read();
		Serial.println(c);

		switch (c)
		{

		case 'c':
		{
			Serial.println("Starting to calibrate");
			myPie.recalibrateMotionSensor();
			Serial.println("Calibration complete");
		}
		break;

		case 'd':
		{
			Serial.println("Sending Midi Event: 64");
			myPie.midiBle.setMidiMsg(0b10010001, 62, 1000);
		}
		break;

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
		case 'f':
		{
			Serial.println("Sending Midi Control Change Event: 64 64");
			myPie.midiBle.setMidiMsg(0b10110000, 64, 64);
		}
		break;

		case 'g':
		{
			Serial.println("Sending Midi Control Change Event: 64 63");
			myPie.midiBle.setMidiMsg(0b10110000, 64, 63);
		}
		break;
	
		case 'l':
		{
			Serial.println("AutoLevel");
			myPie.configOffsetRotation();
		}
		break;

		case 'o':
		{
			Serial.println("Sending Midi Note On Event: 60 64");
			myPie.midiBle.setMidiMsg(0b10010000, 60, 64);
		}
		break;

		case 'p':
		{
			Serial.println("Sending Midi Note Off Event: 60 64");
			myPie.midiBle.setMidiMsg(0b10000000, 60, 64);
		}
		break;

		case 'r':
		{
			Serial.println("Reading Motion Sensor");
			Serial.println(myPie.motionSensor.rotQuat.getRotX());
		}
		break;

		case 's':
		{
			static uint8_t note = 0;
			for (int i = 50; i < 100; i++) {
				myPie.midiBle.setMidiMsg(0b10010000, i, 64);
				myPie.updateDelay(200);
				myPie.midiBle.setMidiMsg(0b10000000, i, 64);
			}
		}
		break;

		case 'z':
		{
			Serial.println("Clear offset");
			myPie.motionSensor.clearOffsetRotation();
		}
		break;
		}
	}
}


void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo)
{
	switch (eventId)
	{
	case TimerTickMngt::TIMER_TICK_100MS:
		/*
		Serial.print("x:");
		Serial.print(myPie.motionSensor.rotQuat.getSinRotX());
		Serial.print("\t");
		Serial.print("y:");
		Serial.println(myPie.motionSensor.rotQuat.getSinRotl
		Y());
		*/
		break;

	case TimerTickMngt::TIMER_TICK_UPDATE:
		break;

	case TimerTickMngt::TIMER_TICK_10S:
		break;

	case TimerTickMngt::TIMER_TICK_S:
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
	{
		static bool isOn = false;
		int16_t tilt_x = myPie.motionSensor.rotQuat.getSinRotX();
		int16_t tilt_y = myPie.motionSensor.rotQuat.getSinRotY();
		int16_t wah;

		if ((tilt_x < -6000) && (tilt_x > -8000)) {
			wah = -tilt_x - 6000;
			wah = wah >> 4;
			myPie.midiBle.setMidiMsg(0b10110000, 43, wah);
		}

		if (tilt_x < -8000)
			myPie.midiBle.setMidiMsg(0b10110000, 43, 127);

		if (tilt_x > -6000)
			myPie.midiBle.setMidiMsg(0b10110000, 43, 0);

	}
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


