/*
 * MySaber - a simple light saber project
 */

#include "BlueberryPie.h"
#include "SigGen.h"
#include "SigLib.h"
#include "ToDiscrete.h"

using namespace SweetMaker;

BlueberryPie myPie;

#define NUM_LIGHTS (StrawberryString::num_lights)
static const SigGen::SAMPLE PROGMEM sawTooth[] = { 0, 256 * NUM_LIGHTS };


class MiniSaberLightControl {
private:

	ColourRGB* output;
	SigGen sineWave;
	StaticGen myStaticGen;
	uint8_t saberHue = 0;
	uint8_t brightness = 0;

public:
	MiniSaberLightControl() {
		sineWave.configSamples(sineWave255, NUM_SAM(sineWave255), 1000, 0);
		myStaticGen.configDuty_256(245);
		myStaticGen.configPeriod_ms(20);
		sineWave.start(0, random(0, 500));
	};

	void configOutput(ColourRGB* light) {
		output = light;
	}

	void update() {
		uint16_t _brightness = 128 + (sineWave.readValue() >> 2);
		_brightness = (_brightness * brightness) >> 8;
		*output = ColourConverter::ConvertToRGB(
			saberHue,
			128 + (myStaticGen.readValue() >> 1),
			_brightness
		);
	}

	void setBrightness(uint8_t _brightness) {
		brightness = _brightness;
	}

	void setColour(uint8_t colour) {
		saberHue = colour;
	}
};

class MiniSaber {
private:
	enum {
		OFF,
		TURNING_ON,
		ON,
		TURNING_OFF,
	}state = OFF;

	MiniSaberLightControl lightControls[NUM_LIGHTS];
    SigGen lightUpSigGen;
	const uint8_t midiChanNum = 1;

public: 
	MiniSaber(BlueberryPie *blueberryPie) {
		lightUpSigGen.configSamples(sawTooth, NUM_SAM(sawTooth), 1500, SigGen::DONT_FINISH_ON_ZERO);

		for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
			lightControls[i].configOutput(&blueberryPie->ledStrip[i]);
		}
	}

	void update() {
		if (state == TURNING_ON) {
			if (lightUpSigGen.isRunning()) {
				int16_t currentVal = lightUpSigGen.readValue();
				for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
					if (currentVal > 256) {
						lightControls[i].setBrightness(255);
						currentVal -= 256;
					}
					else {
						lightControls[i].setBrightness(currentVal);
						currentVal = 0;
					}
				}
			}
			else {
				lightControls[NUM_LIGHTS - 1].setBrightness(0);
				myPie.midiBle.noteOff(midiChanNum, MIDI_A3);
				myPie.midiBle.noteOn(midiChanNum, MIDI_A2);
				state = ON;
			}
		}
		else if (state == TURNING_OFF) {
			if (lightUpSigGen.isRunning()) {
				int16_t currentVal = 256* NUM_LIGHTS - lightUpSigGen.readValue();
				for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
					if (currentVal > 256) {
						lightControls[i].setBrightness(255);
						currentVal -= 256;
					}
					else {
						lightControls[i].setBrightness(currentVal);
						currentVal = 0;
					}
				}
			}
			else {
				lightControls[0].setBrightness(0);
				state = OFF;
				myPie.midiBle.noteOff(midiChanNum, MIDI_A3);
			}
		}

		for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
			lightControls[i].update();
		}
	}

	void turnOn() {
		state = TURNING_ON;
		lightUpSigGen.start(1);
		myPie.midiBle.noteOn(midiChanNum, MIDI_A3);
	}

	void turnOff() {
		state = TURNING_OFF;
		lightUpSigGen.start(1);
		myPie.midiBle.noteOff(midiChanNum, MIDI_A2);
		myPie.midiBle.noteOn(midiChanNum, MIDI_A3);
	}

	void setColour(uint8_t colour) {
		for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
			lightControls[i].setColour(colour);
		}
	}

	void setRotVel(int16_t rotVel) {
		if (state != OFF) {
			myPie.midiBle.pitchBendChange(midiChanNum, 0x1fff + rotVel);
		}
	}

}myMiniSaber(&myPie);


enum Orientation {
	NON_SPECIFIC,
	HORIZONTAL,
	VERTICAL_UP,
	VERTICAL_DOWN
};

typedef struct {
	int16_t angleToVertical_16384;
	int16_t angularVelocity_16384;
	int16_t zAxisRotation_16384;
	Orientation orientation;
}SaberOrientation;

/*
 * Captures events from myPie and SweetMaker framework
 */
void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);
void handleMotionSensorReading(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);
void generateOnAndOffEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, SaberOrientation orientation);
void mySaberEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, SaberOrientation orientation);

#define SABER_EVENT_TURN_ON (IEventHandler::USER + 1)
#define SABER_EVENT_TURN_OFF (IEventHandler::USER + 2)
#define SABER_EVENT_START_COLOUR_SELECTION (IEventHandler::USER + 3)
#define SABER_EVENT_FINISH_COLOUR_SELECTION (IEventHandler::USER + 4)
#define SABER_EVENT_INCREMENT_COLOUR (IEventHandler::USER + 5)
#define SABER_EVENT_DECREMENT_COLOUR (IEventHandler::USER + 6)
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
}

/*
 * Called repeatedly after setup.
 */
void loop()
{
	// This function drives updates for LEDs and everything in the Sweetmaker framework 
	myPie.update();
	myMiniSaber.update();

	// Check for any input on the Serial Port (only relevant when connected to computer)
	handleSerialInput();
}

/*
* This responds to some generic events before calling more focused handler
*/
void myEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo)
{
	switch (eventId)
	{
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

		// These events are unused but handy for debug
	case TimerTickMngt::TIMER_TICK_100MS:
	case TimerTickMngt::TIMER_TICK_S:
	case TimerTickMngt::TIMER_TICK_UPDATE:
	case TimerTickMngt::TIMER_TICK_10S:
	default:
		break;
	}
	handleMotionSensorReading(eventId, srcRef, eventInfo);
}

/*
 * handleMotionSensorReading - called every 10ms when a new motion sensor reading
 *                                is available. Interprets the rotational orientaion
 *                                data and identifies when to update lights and play
 *                                notes.
 */
void handleMotionSensorReading(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo) {

	SaberOrientation orientation = { 0,0, Orientation::NON_SPECIFIC };
	if (eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) {

		/*
		 * Start by manipulating orientation data into meaningful representation
		 */

		 /* Calculate angle from vetical */
		Quaternion_16384 vertical = Quaternion_16384(0, 0, 0, 16384);
		double cos_angleToVertical = (double)myPie.motionSensor.gravity.dotProduct(&vertical) / 16384;
		orientation.angleToVertical_16384 = (int16_t)(acos(cos_angleToVertical) * 0x8000 / M_PI);

		double x = myPie.motionSensor.gravity.x;
		double y = myPie.motionSensor.gravity.y;
		double sin_orientation = x / sqrt(x * x + y * y);
		orientation.zAxisRotation_16384 = (int16_t)(asin(sin_orientation) * 0x8000 / M_PI);

		/* Calculate crude rotational velocity */
		orientation.angularVelocity_16384 = 16383 - myPie.motionSensor.rotQuatDelta.r;
		orientation.angularVelocity_16384 *= 200;

		/* Detect specific orientations of note */
		if (orientation.angleToVertical_16384 < 1500)
			orientation.orientation = Orientation::VERTICAL_UP;
		else if (orientation.angleToVertical_16384 < 16384 - 500)
			orientation.orientation = Orientation::NON_SPECIFIC;
		else if (orientation.angleToVertical_16384 < 16384 + 500)
			orientation.orientation = Orientation::HORIZONTAL;
		else if (orientation.angleToVertical_16384 < 32768 - 2500)
			orientation.orientation = Orientation::NON_SPECIFIC;
		else
			orientation.orientation = Orientation::VERTICAL_DOWN;
	}
	generateOnAndOffEvents(eventId, srcRef, eventInfo, orientation);
}

void generateOnAndOffEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, SaberOrientation orientation)
{
	static Timer onOffTimer(1000, 1);
	static enum {
		OFF,
		OFF_VERTICAL_UP,
		OFF_VERTICAL_DOWN,
		OFF_HORIZONTAL,
		ON,
		ON_VERTICAL_UP,
		OFF_RETURN_TO_HORIZONTAL,
		COLOUR_SELECTION,
	}state = OFF;

	switch (state) {
	case OFF: {
		if ((eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) &&
			(orientation.orientation == Orientation::VERTICAL_UP)) {
			state = OFF_VERTICAL_UP;
			onOffTimer.startTimer(2000, 0);
		}
	}
	break;
	case OFF_VERTICAL_UP: {
		if ((eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) &&
			(orientation.orientation == Orientation::VERTICAL_DOWN)) {
			state = OFF_VERTICAL_DOWN;
			onOffTimer.startTimer(2000, 0);
		}
		else if (eventId == TimerTickMngt::TIMER_EXPIRED) {
			state = OFF;
		}
	}
	break;
	case OFF_VERTICAL_DOWN: {
		if ((eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) &&
			(orientation.orientation == Orientation::HORIZONTAL)) {
			state = ON;
			generateColourChangeEvents(SABER_EVENT_TURN_ON, 0, 0, orientation);
			onOffTimer.stopTimer();
		}
		else if (eventId == TimerTickMngt::TIMER_EXPIRED) {
			state = OFF;
		}
	}
    break;
	case ON: {
		if ((eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) &&
			(orientation.orientation == Orientation::VERTICAL_UP)) {
			state = ON_VERTICAL_UP;
			onOffTimer.startTimer(1500, 0);
		}
	}
	break;
	case ON_VERTICAL_UP: {
		if ((eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) &&
			(orientation.orientation != Orientation::VERTICAL_UP)) {
			state = ON;
			onOffTimer.stopTimer();
		}
		else if (eventId == TimerTickMngt::TIMER_EXPIRED) {
			generateColourChangeEvents(SABER_EVENT_TURN_OFF, 0, 0, orientation);
			state = OFF_RETURN_TO_HORIZONTAL;
			onOffTimer.startTimer(4000, 0);
		}
	}
    break;
	case OFF_RETURN_TO_HORIZONTAL: {
		if ((eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) &&
			(orientation.orientation != Orientation::VERTICAL_UP)) {
			onOffTimer.stopTimer();
			state = OFF;
		}
		else if (eventId == TimerTickMngt::TIMER_EXPIRED) {
			generateColourChangeEvents(SABER_EVENT_START_COLOUR_SELECTION, 0, 0, orientation);
			state = COLOUR_SELECTION;
		}
	}
    break;
	case COLOUR_SELECTION: {
		if ((eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) &&
			(orientation.orientation == Orientation::HORIZONTAL)) {
			generateColourChangeEvents(SABER_EVENT_FINISH_COLOUR_SELECTION, 0, 0, orientation);
			state = ON;
		}
	}
    break;
	}
	generateColourChangeEvents(eventId, srcRef, eventInfo, orientation);
}

void generateColourChangeEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, SaberOrientation orientation) {
	static enum {
		IDLE,
		NEUTRAL_POSITION,
		RETURNING_TO_NEUTRAL,
	}state = IDLE;
	const uint16_t quant_step_size = 6000;
	static ToDiscrete quantizer(quant_step_size, 1000);

	if (eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) {
		quantizer.writeValue(orientation.zAxisRotation_16384 + (quant_step_size>>1));
		Serial.print(state);
		Serial.print(" ");
		Serial.print(quantizer.current_continuous_value);
		Serial.print(" ");
		Serial.println(quantizer.current_discrete_value);
	}

	switch (state) {
	case IDLE: {
		if (eventId == SABER_EVENT_START_COLOUR_SELECTION) {
			quantizer.start(orientation.zAxisRotation_16384);
			state = NEUTRAL_POSITION;
		}
	}
    break;

	case NEUTRAL_POSITION: {
		if (eventId == SABER_EVENT_FINISH_COLOUR_SELECTION) {
			state = IDLE;
		}
		else if (eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) {
			if (quantizer.current_discrete_value > 0) {
				mySaberEventHandler(SABER_EVENT_INCREMENT_COLOUR, 0, 0, orientation);
				state = RETURNING_TO_NEUTRAL;
			}
			if (quantizer.current_discrete_value < 0) {
				mySaberEventHandler(SABER_EVENT_DECREMENT_COLOUR, 0, 0, orientation);
				state = RETURNING_TO_NEUTRAL;
			}
		}
	}
	break;

	case RETURNING_TO_NEUTRAL: {
		if (eventId == SABER_EVENT_FINISH_COLOUR_SELECTION) {
			state = IDLE;
		}
		else if (eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) {
			if (quantizer.current_discrete_value == 0) {
				state = NEUTRAL_POSITION;
			}
		}
	}
    break;
	}
	mySaberEventHandler(eventId, srcRef, eventInfo, orientation);
}

/*
* This function focuses on controlling the Saber
*/
void mySaberEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, SaberOrientation orientation)
{
	static uint8_t currentColourIndex = 0;
	const uint8_t saberColours[] = { 0, 80, 150, 170, 200 };
	const uint8_t numColours = 5;
	switch (eventId)
	{
		/* This function is the main */
	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
		myMiniSaber.setRotVel(orientation.angularVelocity_16384);
		break;

	case SABER_EVENT_TURN_ON:
		myMiniSaber.turnOn();
		break;

	case SABER_EVENT_TURN_OFF:
		myMiniSaber.turnOff();
		break;

	case SABER_EVENT_START_COLOUR_SELECTION:
		myMiniSaber.turnOn();
		break;

	case SABER_EVENT_FINISH_COLOUR_SELECTION:
		break;

	case SABER_EVENT_INCREMENT_COLOUR: {
		currentColourIndex = currentColourIndex == numColours ? 0 : currentColourIndex + 1;
		myMiniSaber.setColour(saberColours[currentColourIndex]);
		break;
	}
	case SABER_EVENT_DECREMENT_COLOUR: {
		currentColourIndex = currentColourIndex == 0 ? numColours : currentColourIndex - 1;
		myMiniSaber.setColour(saberColours[currentColourIndex]);
		break;
	}

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


