/*
 * LOLIN D32 PRO
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
 */

#include "BlueberryPie.h"
#include "ToDiscrete.h"

using namespace SweetMaker;

BlueberryPie myPie;
ToDiscrete * myToDiscrete;

void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);

void setup()
{
	int retVal;
	Serial.begin(112500);
	Serial.println("Welcome to Keys");

	myPie.configEventHandlerCallback(myEventHandler);
	retVal = myPie.init();
	if (retVal != 0)
	{
		Serial.println("myPie init failure");
	}

	myToDiscrete = new ToDiscrete(100, 10);
	Serial.println("Leaving setup");
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

		case 'z':
		{
			Serial.println("Clear offset");
			myPie.motionSensor.clearOffsetRotation();
		}
		break;
		case 'c':
		{
			Serial.println("Starting to calibrate");
			myPie.recalibrateMotionSensor();
			Serial.println("Calibration complete");
		}
		break;

		case 'l':
		{
			Serial.println("AutoLevel");
			myPie.configOffsetRotation();
		}
		break;

		case 'd':
		{
			Serial.println("Sending Midi Event: 64");
			myPie.midiBle.setMidiMsg(0b10010001, 62, 1000);
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
	{
		Serial.println("MotionSensor Ready");
//		int16_t tilt_x = myPie.motionSensor.rotQuat.getRotX();
		myToDiscrete->start(0);
//		Serial.println(myToDiscrete->current_discrete_value);
	}
	break;

	case MotionSensor::MOTION_SENSOR_RUNTIME_ERROR:
		Serial.println("MotionSensor Run Time Error");
		break;

	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
	{
		Serial.println("MOTION_SENSOR_NEW_SMPL_RDY");

		int16_t tilt_x = myPie.motionSensor.rotQuat.getRotX();
		myToDiscrete->writeValue((uint32_t)tilt_x);
	}
	break;

	case MidiBle::BME_CONNECT:
		Serial.println("BLE Connected");
		break;

	case MidiBle::BME_DISCONNECT:
		Serial.println("BLE Disconnected");
		break;

	case ToDiscrete::NEW_VALUE:
		Serial.print("To Discrete: New Value:");
		Serial.println(myToDiscrete->current_discrete_value);
		break;

	default:
		Serial.print("Event: ");
		Serial.println(eventId);
		break;
	}
}


