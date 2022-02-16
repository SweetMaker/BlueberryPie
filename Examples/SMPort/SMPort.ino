#include <BLEDevice.h>
#include <Wire.h>
#include <MotionSensor.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <SweetMaker.h>
#include "Quaternion_16384.h"
#include "Updater.h"
#include "MidiBle.h"

using namespace SweetMaker;

void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);
void processMotionSensorReading();


MotionSensor myMotionSensor;
MotionSensor::CALIBRATION cal;

MidiBle myMidiBle;



void setup()
{
	Serial.begin(115200);
	Serial.println("Hello World");
	TimerTickMngt::getTimerMngt();
	EventMngr::getMngr()->configCallBack(myEventHandler);

	pinMode(17, OUTPUT);
	digitalWrite(17, HIGH);
	pinMode(16, OUTPUT);
	digitalWrite(16, LOW);

	MotionSensor::CALIBRATION cal = { -883, 456, 852, 70,19,-5 };
	while(myMotionSensor.init(&cal))
		// repeat init until it works!
		;

//	myMotionSensor.runSelfCalibrate(&cal);
	myMidiBle.setup();
}

void loop()
{
	static unsigned long lastTime_ms = millis();
	unsigned long elapsedTime_ms = millis() - lastTime_ms;
	if (elapsedTime_ms > 1000) {
//		Serial.println(millis());
		lastTime_ms = millis();
	}
	SweetMaker::AutoUpdateMngr::getUpdater()->update();
}


void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo)
{
	switch (eventId)
	{
	case TimerTickMngt::TIMER_TICK_10S:
		Serial.print(myMotionSensor.rotQuat.getSinRotX());
		Serial.print("\t");
		Serial.print(myMotionSensor.rotQuat.getSinRotY());
		Serial.print("\t");
		Serial.print(myMotionSensor.rotQuat.getSinRotZ());
		Serial.println();
		break;

	case TimerTickMngt::TIMER_TICK_S:

		if (Serial.available())
		{
			char c = Serial.read();
			switch (c)
			{
			case 'c':
			{
				myMotionSensor.runSelfCalibrate(&cal);
				break;
			}

			case 'r':
			{
				myMotionSensor.clearOffsetRotation();
				RotationQuaternion_16384 oq(&myMotionSensor.rotQuat);
				oq.conjugate();
				myMotionSensor.setOffsetRotation(&oq);
				break;
			}

			case '1':
			{
				myMidiBle.setMidiMsg(0x90, 0x3c, 127);
				break;
			}

			case '2':
			{
				myMidiBle.setMidiMsg(0x90, 0x3d, 127);
				break;
			}

			case '3':
			{
				myMidiBle.setMidiMsg(0x90, 0x3e, 127);
				break;
			}

			case 'x':
			{
				myMidiBle.setMidiMsg(0x80, 0x3c, 0);
				break;
			}

			case 'q':
			{
				myMotionSensor.clearOffsetRotation();
			}

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
		processMotionSensorReading();
		break;

	case MidiBle::BME_CONNECT:
		Serial.println("BLE Connected");
		break;

	case MidiBle::BME_DISCONNECT:
		Serial.println("BLE Disconnected");
		break;

	default:
		break;
	}
}


void processMotionSensorReading()
{
	static uint8_t state = 0;
	switch (state)
	{
	case 0:
		if (myMotionSensor.rotQuat.getSinRotX() > 5000)
		{
			myMidiBle.setMidiMsg(0x90, 0x3c, 127);
			state = 1;
			Serial.println("Play Note: 0x3c");
		}
		if (myMotionSensor.rotQuat.getSinRotX() < -5000)
		{
			myMidiBle.setMidiMsg(0x90, 0x3d, 127);
			state = 2;
			Serial.println("Play Note: 0x3d");
		}


		break;

	case 1:
		if (myMotionSensor.rotQuat.getSinRotX() < 3000)
			state = 0;
		break;

	case 2:
		if (myMotionSensor.rotQuat.getSinRotX() > -3000)
			state = 0;
		break;

	}


}
