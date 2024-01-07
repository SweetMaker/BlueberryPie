/*
 * LOLIN D32 PRO
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
 */

#include "BlueberryPie.h"
#include "ToDiscrete.h"
#include "SigGen.h"
#include "SigLib.h"

using namespace SweetMaker;

#define NUM_LIGHTS (StrawberryString::num_lights)


uint8_t static const c_minor[] = { 60,62,63,65,67,68,70 };
uint8_t static const hue[] = { 0,40,80,120,160,200,240 };


BlueberryPie myPie;

ToDiscrete zAxisRotationQuantizer(3000, 500); // Used for note selection
ToDiscrete verticalTiltQuantizer(1000, 200);  // Used for controlling playing notes
int16_t zAxisAboutVertical_16384;
int16_t verticalVelocity;

SigGen mySigGen;
SigGen sineWave(sineWave255, NUM_SAM(sineWave255), 200, 0);;
StaticGen myStaticGen;
static const SigGen::SAMPLE PROGMEM flashLowWave[] = { 127, 0, 0, 0, 0, 127 };
static const SigGen::SAMPLE PROGMEM flashHighWave[] = { 127, 200, 200, 200, 200, 127 };

ColourHSV lightStripHSV[NUM_LIGHTS];

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
	Serial.println("Welcome to Bells");

	myPie.configEventHandlerCallback(myEventHandler);

	retVal = myPie.init();
	if (retVal != 0)
	{
		Serial.println("myPie init failure");
	}

	// Setup some starting values for the LEDs
	lightStripHSV[0].setColour(0, 255, 127);
	lightStripHSV[1].setColour(160, 255, 127);
	lightStripHSV[2].setColour(80, 255, 127);
	lightStripHSV[3].setColour(200, 255, 127);
	lightStripHSV[4].setColour(120, 255, 127);

	sineWave.start(); // Used to make light more interesting
}

/*
 * Called repeatedly after setup. 
 */
void loop()
{
	// Update each light from HSV value to RGB - Brightness is set depending on mySigGen
	uint8_t value = mySigGen.isRunning() ? (uint8_t)mySigGen.readValue() : 127; // Used for Lights HSV value (brightness)
	for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
		ColourHSV& hsv = lightStripHSV[i];
		uint8_t hueD = ((uint8_t)sineWave.readValue()) >> 4;
		myPie.ledStrip[i] = ColourConverter::ConvertToRGB(hsv.hue + hueD, hsv.saturation, value);
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
	// Calculate rotation about z axis - used for selecting notes
	double x = myPie.motionSensor.gravity.x;
	double y = myPie.motionSensor.gravity.y;
	double sin_orientation = x / sqrt(x * x + y * y); 
	int16_t zAxisRotation_16384 = (int16_t)(asin(sin_orientation) * 0x8000 / M_PI);
	// Feed this value into quantizer to give note selection with hysteresis
	zAxisRotationQuantizer.writeValue((int32_t)zAxisRotation_16384);

	// Calculate angle from vertical - used for playing notes
	Quaternion_16384 vertical = Quaternion_16384(0, 0, 0, 16384);
	double cos_angleToVertical= (double)myPie.motionSensor.gravity.dotProduct(&vertical) / 16384;
	int16_t angleToVertical_16384 = (int16_t)(acos(cos_angleToVertical) * 0x8000 / M_PI);
	verticalTiltQuantizer.writeValue((int32_t)angleToVertical_16384);

	// Calculate rotation of z axis about vertical - used for modulation 
	Quaternion_16384 zAxis = Quaternion_16384(0, 0, 0, 16384);
	myPie.motionSensor.rotQuat.rotate(&zAxis);
	x = zAxis.x;
	y = zAxis.y;
	sin_orientation = x / sqrt(x * x + y * y);
	zAxisAboutVertical_16384 = (int16_t)(asin(sin_orientation) * 0x8000 / M_PI);
	
	// Calculate rotational velocity - WRT vertical
	static int16_t previousAngleToVertical = 0;
	verticalVelocity = angleToVertical_16384 - previousAngleToVertical;
	previousAngleToVertical = angleToVertical_16384;

	/*
	* And now lets respond to what is happening in a stateful manor
	*/
	BELLS_handle_event();
}


/*
 * bellsStateMachine - Controls behaviour of Bells
 */
typedef enum {
	MUTE = 0,
	DAMPING = 1,
	NOTE_SELECTION = 2,
	COLOUR_LOCKIN = 3,
	PRE_STRIKE = 4,
	POST_STRIKE = 5,
}BELLS_STATE;

typedef enum {
	MUTE_ZONE = 0, // Used for muting a played note
	DAMP_ZONE = 1, // Used for damping a played note
	NOTE_SELECTION_ZONE = 2, // Used for selecting the next note
	COLOUR_LOCKIN_ZONE = 3, // Not currently used
	STRIKE_ZONE = 4, // Used for controlling the playing of notes
}BELLS_ZONE;

/* This maps the quantized tilt to 'zones' used by Bells */
uint8_t convertTiltToZone(int16_t tilt) {
	static int16_t zoneMap[] = { 
		MUTE_ZONE ,
		DAMP_ZONE ,
		NOTE_SELECTION_ZONE,
		NOTE_SELECTION_ZONE,
		NOTE_SELECTION_ZONE,
		NOTE_SELECTION_ZONE,
		NOTE_SELECTION_ZONE,
		NOTE_SELECTION_ZONE,
		STRIKE_ZONE,
		STRIKE_ZONE,
		STRIKE_ZONE,
		STRIKE_ZONE,
		STRIKE_ZONE,
		STRIKE_ZONE,
		STRIKE_ZONE,
		STRIKE_ZONE
	};
	if (tilt < 0) tilt = 0;
	if (tilt >= 16) tilt = 15;
	return (zoneMap[tilt]);
}

void BELLS_handle_event() {
	/*
	* This is the state information we use
	*/
	static BELLS_STATE state;
	static uint8_t currentNote = 0;
	static uint8_t nextNote = 0;
	static int16_t lastOrientation = 0;
	static int16_t maxVerticalVelocity = 0;
	static int16_t modulationNullPosition = 0;

	int16_t tilt = verticalTiltQuantizer.current_discrete_value;
	uint8_t zone = convertTiltToZone(tilt);

	switch (state) {
	case NOTE_SELECTION: {
		if (zone == STRIKE_ZONE) {
			Serial.println("Has started strike");
			state = PRE_STRIKE;
			mySigGen.configSamples(flashHighWave, NUM_SAM(flashHighWave), 200, SigGen::DONT_FINISH_ON_ZERO);
			mySigGen.start(1);
			sineWave.stop();
			maxVerticalVelocity = 0;
			break;
		}
		if ((zone == DAMP_ZONE) || (zone == MUTE_ZONE)) {
			Serial.println("Has started damping");
			state = DAMPING;
			break;
		}
		int16_t orientation = zAxisRotationQuantizer.current_discrete_value;
		if ((orientation >= -3) && (orientation <= 3)) {
			nextNote = c_minor[orientation + 3];
			uint8_t sat = 255;
			if ((zAxisRotationQuantizer.in_step_value < 0) || (zAxisRotationQuantizer.in_step_value > 3000))
				sat = 168;
			for (uint8_t i = 0; i < NUM_LIGHTS; i++)
				lightStripHSV[i].setColour(hue[orientation + 3], sat, 127);
		}
	}
    break;

	case PRE_STRIKE: {
		if (verticalVelocity > maxVerticalVelocity) {
			maxVerticalVelocity = verticalVelocity;
		}

		if (verticalVelocity < -20) {
			Serial.println("Has struck:");
			uint8_t velocity = (uint8_t)((uint16_t)maxVerticalVelocity >> 3);
			if (velocity > 127) velocity = 127;
			myPie.midiBle.setMidiMsg(0b10000000, currentNote, 64);
			myPie.midiBle.setMidiMsg(0b10010000, nextNote, velocity);
			currentNote = nextNote;
			mySigGen.configSamples(flashLowWave, NUM_SAM(flashLowWave), 200, SigGen::DONT_FINISH_ON_ZERO);
			mySigGen.start(1);
			modulationNullPosition = zAxisAboutVertical_16384;
			state = POST_STRIKE;
		}
	}
	break;
				    
	case POST_STRIKE: {
		if (zone == NOTE_SELECTION_ZONE) {
			Serial.println("Has started note selection");
			sineWave.start();
			state = NOTE_SELECTION;
			break;
		}
		// Modulate note
		int16_t rawModulation = abs(zAxisAboutVertical_16384 - modulationNullPosition);
		Serial.print(zAxisAboutVertical_16384); Serial.print(" ");
		Serial.print(modulationNullPosition); Serial.print(" ");
		Serial.print(rawModulation); Serial.println(" ");
		if ((rawModulation > 3000) && (rawModulation < 3000 + 2048)) {
			uint8_t modulation = (uint8_t)(((uint16_t)(rawModulation - 3000)) >> 4);
			myPie.midiBle.setMidiMsg(0b11100000, 1, modulation);
		}
		if (rawModulation < 1000) {
			myPie.midiBle.setMidiMsg(0b11100000, 1, 64);
		}
	}
    break;

	case DAMPING: {
		if (zone == DAMP_ZONE) {
			Serial.println(verticalTiltQuantizer.in_step_value);
			// need to dampen note
		}
		else if (zone == MUTE_ZONE) {
			Serial.println("Has muted");
			myPie.midiBle.setMidiMsg(0b10000000, currentNote, 64);
			state = MUTE;
		}
		else {
			Serial.println("Has started note selection");
			state = NOTE_SELECTION;
		}
	}
    break;

	case MUTE: {
		if (zone != MUTE) {
			Serial.println("Has stopped muting");
			state = DAMPING;
		}
	}
    break;
	}
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
		zAxisRotationQuantizer.start(0);
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
		break;

	// These events are unused but handy for debug
	case ToDiscrete::NEW_VALUE:
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
