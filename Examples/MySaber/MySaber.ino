/*
 * MySaber - a simple light saber project
 */

/*
 * LOLIN D32 PRO
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
 */
#include "BlueberryPie.h"
#include "SigGen.h"
#include "SigLib.h"

using namespace SweetMaker;

#define NUM_LIGHTS (StrawberryString::num_lights)

BlueberryPie myPie;
uint8_t saberHue = 0;

class MySaberLightControl {
public:
	MySaberLightControl() {
		sineWave.configSamples(sineWave255, NUM_SAM(sineWave255), 1000, 0);
		myStaticGen.configDuty_256(225);
		myStaticGen.configPeriod_ms(40);
		sineWave.start(0, random(0,1000));
	};

	void configOutput(ColourRGB* light) {
		output = light;
	}
	
	void update() {
		*output = ColourConverter::ConvertToRGB(
			saberHue,
			128 + (myStaticGen.readValue() >> 1),
			128 + (sineWave.readValue() >> 1)
		);
	}

private:
	ColourRGB* output;
	SigGen sineWave;
	StaticGen myStaticGen;
};

MySaberLightControl lightControls[NUM_LIGHTS];
#define MIDI_CHAN_NUM (1)

/*
 * Captures events from myPie and SweetMaker framework
 */
void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);

/*
 * Runs once when the system starts up.
 */
void setup()
{
	int retVal;
	Serial.begin(112500); // set the baud rate to 112500 on PC
	Serial.println("Welcome to MiniSaber");

	myPie.configEventHandlerCallback(myEventHandler);

	retVal = myPie.init();
	if (retVal != 0) {
		Serial.println("myPie init failure");
	}

	for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
		lightControls[i].configOutput(&myPie.ledStrip[i]);
	}
}

/*
 * Called repeatedly after setup.
 */
void loop()
{
	// Update each lightControl
	for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
		lightControls[i].update();
	}

	// This function drives updates for LEDs and everything in the Sweetmaker framework 
	myPie.update();

	// Check for any input on the Serial Port (only relevant when connected to computer)
	handleSerialInput();
}

/*
 * handleMotionSensorNewSmplRdy - called every 10ms when a new motion sensor reading
 *                                is available. Interprets the rotational orientaion
 *                                data and identifies when to update lights and play
 *                                notes.
 */
void handleMotionSensorNewSmplRdy(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo) {
	/*
	 * Start by manipulating orientation data into meaningful representation
	 */
	//myPie.motionSensor.rotQuatDelta.printQ();
	int16_t rotVel = 16383 - myPie.motionSensor.rotQuatDelta.r;
	rotVel *= 200;
	Serial.println(rotVel);
	myPie.midiBle.pitchBendChange(MIDI_CHAN_NUM, 0x1fff + rotVel);
}


/*
* Capture and respond to SweetMaker events
*/
void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo)
{
	switch (eventId)
	{
		/* This functioin is the main */
	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
		handleMotionSensorNewSmplRdy(eventId, srcRef, eventInfo);
		break;


	case MotionSensor::MOTION_SENSOR_INIT_ERROR:
		Serial.println("MotionSensor Init Error");
		break;

	case MotionSensor::MOTION_SENSOR_READY:
	{
		Serial.println("MotionSensor Ready");
	}
	break;

	case MotionSensor::MOTION_SENSOR_RUNTIME_ERROR:
		Serial.println("MotionSensor Run Time Error");
		break;

	case MidiBle::BME_CONNECT:
		Serial.println("BLE Connected");
		break;

	case MidiBle::BME_DISCONNECT:
		Serial.println("BLE Disconnected");
		break;

	case TimerTickMngt::TIMER_TICK_S:
		myPie.midiBle.noteOn(MIDI_CHAN_NUM, MIDI_B2);
		break;

		// These events are unused but handy for debug
	case TimerTickMngt::TIMER_TICK_100MS:
	case TimerTickMngt::TIMER_TICK_UPDATE:
	case TimerTickMngt::TIMER_TICK_10S:
	default:
		break;
	}
}

/* This supports various management functions as shown below */
void handleSerialInput() {
	if (Serial.available()) {
		char c = Serial.read();
		Serial.println(c);

		switch (c) {

		case 'c': {
			// Calibrates the motionSensor and stores result in EEPROM
			Serial.println("MotionSensor must be level and stationary");
			Serial.println("Starting to calibrate");
			myPie.recalibrateMotionSensor();
			Serial.println("Calibration complete");
		}
				break;

		case 'l': {
			// Configures the motionSensor rotation offset to believe it is level
			// Stores the configuration in EEPROM
			Serial.println("AutoLevel");
			myPie.configOffsetRotation();
		}
				break;

		case 'z': {
			// Removes any rotation offset from the motionSensor
			Serial.println("Clear offset");
			myPie.motionSensor.clearOffsetRotation();
		}
				break;
		}
	}
}


