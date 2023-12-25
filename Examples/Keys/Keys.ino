/*
 * LOLIN D32 PRO
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
 */

#include "BlueberryPie.h"
#include "ToDiscrete.h"

using namespace SweetMaker;

BlueberryPie myPie;
ToDiscrete * myToDiscrete;
int16_t tilt_y;

void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);

void setup()
{
	int retVal;
	Serial.begin(112500);
	Serial.println("Welcome to Keys");

	myPie.configEventHandlerCallback(myEventHandler);
	myToDiscrete = new ToDiscrete(1000, 100);

	retVal = myPie.init();
	if (retVal != 0)
	{
		Serial.println("myPie init failure");
	}

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
		myToDiscrete->start(0);
	}
	break;

	case MotionSensor::MOTION_SENSOR_RUNTIME_ERROR:
		Serial.println("MotionSensor Run Time Error");
		break;

	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
	{
		//Serial.println("MOTION_SENSOR_NEW_SMPL_RDY");
		int16_t tilt_x = myPie.motionSensor.rotQuat.getRotX();
		tilt_y = myPie.motionSensor.rotQuat.getRotY();
		myToDiscrete->writeValue((int32_t)tilt_x);
		if (tilt_y > 0) {
			uint8_t volume = 128 - (tilt_y >> 7);
			myPie.midiBle.setMidiMsg(0b10110000, 7, volume);
		}

		Quaternion_16384 vertical = Quaternion_16384(0, 0, 0, 16384);
		int16_t howVertical = myPie.motionSensor.gravity.dotProduct(&vertical);
		
		static boolean isVertical = false;
		if (howVertical > 15000 && !isVertical) {
			isVertical = true;
			myPie.ledStrip[0].setColour(128, 128, 128);
		}
		else if (howVertical < 14500 && isVertical) {
			isVertical = false;
			myPie.ledStrip[0].setColour(0, 0, 0);
		}
	}
	break;

	case MidiBle::BME_CONNECT:
		Serial.println("BLE Connected");
		break;

	case MidiBle::BME_DISCONNECT:
		Serial.println("BLE Disconnected");
		break;

	case ToDiscrete::NEW_VALUE:
	{
		int16_t value = myToDiscrete->current_discrete_value;
		Serial.print("To Discrete: New Value:");
		Serial.println(value);

		// C, D, E?, F, G, A?, and B?
		uint8_t static const c_minor[] = { 60,62,63,65,67,68 };
		if ((value >= 0) && (value < 6)) {
			static uint8_t lastNote = 60;
			uint8_t midiNote = c_minor[value];
			myPie.midiBle.setMidiMsg(0b10000000, lastNote, 64);
			myPie.midiBle.setMidiMsg(0b10010000, midiNote, 64);
			lastNote = midiNote;
		}
	}
	break;

	default:
		Serial.print("Event: ");
		Serial.println(eventId);
		break;
	}
}


