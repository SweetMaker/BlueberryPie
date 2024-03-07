/*
 * LOLIN D32 PRO
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
 */

#include <Wire.h>
#include "ToDiscrete.h"
#include "BlueberryPie.h"
#include "ToDiscrete.h"

using namespace SweetMaker;

/*
 * Type definitions
 */

enum SpecificOrientation {
	ZONE_VERTICAL_UP,
	ZONE_CONTINUOUS_CONTROL,   // Used for 'discrete' pedals 
	ZONE_COMMAND_SELECTION,    // Used for selecting between commands
	ZONE_TOGGLE,               // Used for toggling pedals on and off
	ZONE_NON_SPECIFIC,         // Not used for anything
	ZONE_VERTICAL_DOWN,
	HORIZONTAL,                // In a horizontal orientation 
	TILTED_ABOUT_NECK          // rotated about neck
};

#define WAHWAH_EVENT_TURN_ON					(IEventHandler::USER + 1)
#define WAHWAH_EVENT_TURN_OFF					(IEventHandler::USER + 2)
#define WAHWAH_EVENT_ENTERED_CMD_SLCT_MODE		(IEventHandler::USER + 3)
#define WAHWAH_EVENT_LEFT_CMD_SLCT_MODE			(IEventHandler::USER + 4)
#define WAHWAH_EVENT_CMD_SLCT_NEW      			(IEventHandler::USER + 5)
#define WAHWAH_EVENT_CMD_SLCT_NULL     			(IEventHandler::USER + 6)
#define WAHWAH_EVENT_CMD_TOGGLE          	    (IEventHandler::USER + 7)
#define WAHWAH_EVENT_CMD_CONT_MODE_ENTERED		(IEventHandler::USER + 8)

// These are Identifiers for the ToDiscrete instances so we can differentiate events from each
#define ANGLE_ABOUT_VERTICAL_REF	(0)
#define ANGLE_TO_VERTICAL_REF		(1)
#define ROTATION_ABOUT_NECK_REF		(2)

// These are used to help configure the ToDiscrete instances
#define NUM_VERTICAL_SEGMENTS       (16)
#define NUM_HORIZONTAL_SEGMENTS     (8)
#define NUM_NECK_SEGMENTS           (4)

#define NUM_MIDI_CMDS               (5)



// This is used to collate helpful information following a new motion sensor reading

typedef struct {
	ToDiscrete angleAboutVertical{ 0x4000/ NUM_HORIZONTAL_SEGMENTS, 64, ANGLE_ABOUT_VERTICAL_REF };
	ToDiscrete angleToVertical{ 0x8000/ NUM_VERTICAL_SEGMENTS, 64, ANGLE_TO_VERTICAL_REF };
	ToDiscrete rotationAboutNeck{ 0x4000/NUM_NECK_SEGMENTS, 64, ROTATION_ABOUT_NECK_REF };
	SpecificOrientation verticalOrientation;
	SpecificOrientation neckRotationOrientation;
}Orientation;


/*
 * Function Prototypes
 */
void blueberryPieEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);
void handleMotionSensorReading(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo);
void generateWahWahEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, Orientation orientation);
void handleWahWahEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, Orientation orientation);
void printOrientation();

/*
 * Object declaration 
 */
BlueberryPie myPie;
Orientation currentOrientation;

// Maps vertical oritentation to zones used by WahWah
uint8_t verticalZoneMap[NUM_VERTICAL_SEGMENTS] = {
  ZONE_VERTICAL_UP,
  ZONE_CONTINUOUS_CONTROL,
  ZONE_CONTINUOUS_CONTROL,
  ZONE_COMMAND_SELECTION,
  ZONE_COMMAND_SELECTION,
  ZONE_COMMAND_SELECTION,
  ZONE_TOGGLE,
  ZONE_TOGGLE,
  ZONE_NON_SPECIFIC,
  ZONE_NON_SPECIFIC,
  ZONE_NON_SPECIFIC,
  ZONE_NON_SPECIFIC,
  ZONE_NON_SPECIFIC,
  ZONE_NON_SPECIFIC,
  ZONE_VERTICAL_DOWN,
  ZONE_VERTICAL_DOWN,
};


class WahWahLight : AutoUpdate {
public:
	ColourRGB* light;

	SigGen colourGen;
	SigGen saturationGen;
	SigGen brightnessGen;
	
	uint8_t colour;
	uint8_t saturation;
	uint8_t brightness;

	uint8_t* saturationTracker;
	uint8_t* brightnessTracker;

	WahWahLight() {
		colour = 0;
		saturation = 0;
		brightness = 0;

		saturationTracker = NULL;
	};

	void setLight(uint8_t _colour, uint8_t _saturation, uint8_t _brightness) {
		colourGen.stop();
		saturationGen.stop();
		brightnessGen.stop();

		saturationTracker = NULL;
		brightnessTracker = NULL;

		colour = _colour;
		saturation = _saturation;
		brightness = _brightness;
	}

	void update(uint16_t elapsed_time) {
		uint8_t h, s, v;
		h = colourGen.isRunning() ? colourGen.readValue() : colour;

		if (saturationGen.isRunning()) {
			s = saturationGen.readValue();
		}
		else if (saturationTracker) {
			s = 255 - *saturationTracker;
		}
		else {
			s = saturation;
		}

		if (brightnessGen.isRunning()) {
			v = brightnessGen.readValue();
		}
		else if (brightnessTracker) {
			v = (255 - *brightnessTracker) >> 1;
			v = v > 10 ? v : 10;
		}
		else {
			v = brightness;
		}

		*light = ColourConverter::ConvertToRGB(h, s, v);
	}
};

WahWahLight wahWahLights[myPie.num_lights];
bool pedalOn[NUM_MIDI_CMDS];


// Called once at startup
void setup()
{
	int retVal;
	Serial.begin(112500);
	Serial.println("Welcome to Guitar Wahwah");

	myPie.configEventHandlerCallback(blueberryPieEventHandler);
	retVal = myPie.init();
	if (retVal != 0)
	{
		Serial.println("myPie init failure");
	}
	Serial.println("myPie init complete");

	currentOrientation.angleToVertical.start(0);
	currentOrientation.angleAboutVertical.start(0);
	currentOrientation.rotationAboutNeck.start(0);
	currentOrientation.verticalOrientation = ZONE_NON_SPECIFIC;
	currentOrientation.neckRotationOrientation = ZONE_NON_SPECIFIC;

	for (int i = 0; i < myPie.num_lights; i++) {
		wahWahLights[i].light = myPie.ledStrip + i;
	}

	for (int i = 0; i > NUM_MIDI_CMDS; i++) {
		pedalOn[i] = false;
	}
}

// Called repeatedly after setup
// As this sketch is event driven this code needs to do very little
// other than call myPie.update and handle any Serial port events
void loop()
{
	myPie.update();
	handleSerialInput();
}

// This event handler does some housekeeping in that it is checking for some 
// stock events and outputting some diagnostics where helpful
void blueberryPieEventHandler(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo)
{
	switch (eventId)
	{
	case MotionSensor::MOTION_SENSOR_INIT_ERROR:
		Serial.println("MotionSensor Init Error");
		break;

	case MotionSensor::MOTION_SENSOR_READY:
		Serial.println("MotionSensor Ready");
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
		Serial.println(currentOrientation.angleAboutVertical.distance_to_mid_255);
		break;

	case TimerTickMngt::TIMER_TICK_100MS:
	case TimerTickMngt::TIMER_TICK_UPDATE:
	case TimerTickMngt::TIMER_TICK_10S:
	default:
		break;
	}

	handleMotionSensorReading(eventId, srcRef, eventInfo);
}

/*
 * handleMotionSensorReading - focuses on the MOTION_SENSOR_NEW_SMPL_RDY event every 10ms 
 *                                when a new motion sensor reading is available. 
 *                                Interprets the rotational orientaion
 *                                data and puts it into meaning for this app
 * 
 * angle about Vertical   : guitar rotates North, East, South, West
 * neck angle to Vertical : guitar neck pivots between vertical and horizontal
 * rotation of neck       : guitar rotates along axis of fretboard
 */
void handleMotionSensorReading(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo) {

	if (eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) {

		 /* Calculate angle to vertical - derived from relative direction of gravity*/
		Quaternion_16384 vertical = Quaternion_16384(0, 0, 0, 0x4000);
		double cos_angleToVertical = (double)myPie.motionSensor.gravity.dotProduct(&vertical) / 0x4000;
		currentOrientation.angleToVertical.writeValue(acos(cos_angleToVertical) * 0x8000 / M_PI);

		/* Calculate rotation of neck - derived from relative direction of gravity*/
		double x = myPie.motionSensor.gravity.x;
		double y = myPie.motionSensor.gravity.y;
		double sin_orientation = x / sqrt(x * x + y * y);
		currentOrientation.rotationAboutNeck.writeValue(asin(sin_orientation) * 0x8000 / M_PI);

		/* Calculate horizontal rotation of guitar about vertical */
		Quaternion_16384 zAxis = Quaternion_16384(0, 0, 0, 0x4000);
		myPie.motionSensor.rotQuat.rotate(&zAxis);
		x = zAxis.x;
		y = zAxis.y;
		sin_orientation = x / sqrt(x * x + y * y);
		currentOrientation.angleAboutVertical.writeValue(asin(sin_orientation) * 0x8000 / M_PI);


		// printOrientation();
	}

	/*
	 * We capture this event perculating through here to make sure the verticalOrientation is properly set
	 */
	if (eventId == ToDiscrete::NEW_VALUE){
		if (srcRef == ANGLE_TO_VERTICAL_REF) {
			currentOrientation.verticalOrientation = (SpecificOrientation)verticalZoneMap[currentOrientation.angleToVertical.current_discrete_value];
		}
		if (srcRef == ROTATION_ABOUT_NECK_REF) {
			currentOrientation.neckRotationOrientation = ZONE_NON_SPECIFIC;
			if ((currentOrientation.rotationAboutNeck.current_discrete_value > 1) || (currentOrientation.rotationAboutNeck.current_discrete_value < -2))
				currentOrientation.neckRotationOrientation = TILTED_ABOUT_NECK;
		}
	}

	generateWahWahEvents(eventId, srcRef, eventInfo, currentOrientation);
}

// This follows on from handleMotionSensorReading and now interprets the orientation
// data specific to this sketch and the current state.
void generateWahWahEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, Orientation orientation)
{
    // Maintains the current state
	static enum states {
		OFF,
		ON,
		CMD_SELECTION_MODE,
		CMD_TOGGLE_MODE,
		CMD_CONTINOUS_CONTROL_MODE
	} state = OFF;
	static boolean isTilted = false;
	static uint8_t selectedCommand;

	/*
	 * The state machine only changes when one of the ToDiscrete's changes value
	 */
	if(eventId == ToDiscrete::NEW_VALUE) {

		/* Start by managing ON and OFF state - this overrides other handling*/
		if (srcRef == ROTATION_ABOUT_NECK_REF && orientation.verticalOrientation != ZONE_VERTICAL_UP) {
			if (!isTilted && orientation.neckRotationOrientation == TILTED_ABOUT_NECK) {
				isTilted = true;
				if (state == OFF) {
					handleWahWahEvents(WAHWAH_EVENT_TURN_ON, 0, 0, orientation);
					state = ON;
				}
				else {
					handleWahWahEvents(WAHWAH_EVENT_TURN_OFF, 0, 0, orientation);
					state = OFF;
				}
			}
			else if (isTilted && orientation.neckRotationOrientation != TILTED_ABOUT_NECK) {
				isTilted = false;
			}
		}

		/* Priniciple state machine */
		switch (state) {
    		case ON: {
		    	if (orientation.verticalOrientation == ZONE_COMMAND_SELECTION) {
					printOrientation();
			    	currentOrientation.angleAboutVertical.stop();
				    myPie.motionSensor.resetHorizontalOrientation();
				    state = CMD_SELECTION_MODE;
				    selectedCommand = 0;
				    handleWahWahEvents(WAHWAH_EVENT_ENTERED_CMD_SLCT_MODE, 0, 0, orientation);
			    }
		    }
			break;

		  case CMD_SELECTION_MODE: {

			  if (srcRef == ANGLE_ABOUT_VERTICAL_REF) {
				  int16_t value = orientation.angleAboutVertical.current_discrete_value;
				  if ((value <= 0) && (value > -NUM_MIDI_CMDS)) {
					  selectedCommand = -value;
					  handleWahWahEvents(WAHWAH_EVENT_CMD_SLCT_NEW, 0, selectedCommand, orientation);
				  }
				  else {
					  selectedCommand = NUM_MIDI_CMDS; // invalid value;
					  handleWahWahEvents(WAHWAH_EVENT_CMD_SLCT_NULL, 0, 0, orientation);
				  }
			  }

			  if ((srcRef == ANGLE_TO_VERTICAL_REF) && (orientation.verticalOrientation != ZONE_COMMAND_SELECTION)) {
				  if (selectedCommand == NUM_MIDI_CMDS) {
					  handleWahWahEvents(WAHWAH_EVENT_LEFT_CMD_SLCT_MODE, 0, selectedCommand, orientation);
					  state = ON;
					  break;
				  }
				  if (orientation.verticalOrientation == ZONE_CONTINUOUS_CONTROL) {
					  handleWahWahEvents(WAHWAH_EVENT_CMD_CONT_MODE_ENTERED, 0, selectedCommand, orientation);
					  state = CMD_CONTINOUS_CONTROL_MODE;
					  break;
				  }
				  if (orientation.verticalOrientation == ZONE_TOGGLE) {
					  handleWahWahEvents(WAHWAH_EVENT_CMD_TOGGLE, 0, selectedCommand, orientation);
					  state = CMD_TOGGLE_MODE;
					  break;
				  }
			  }
		  }
		  break;

		  case CMD_TOGGLE_MODE: {

			  if (srcRef == ANGLE_TO_VERTICAL_REF && orientation.verticalOrientation == ZONE_COMMAND_SELECTION) {
				  handleWahWahEvents(WAHWAH_EVENT_CMD_SLCT_NEW, 0, selectedCommand, orientation);
				  state = CMD_SELECTION_MODE;
				  break;
			  }
		  }
		  break;

		  case CMD_CONTINOUS_CONTROL_MODE: {
			  if (srcRef == ANGLE_TO_VERTICAL_REF && orientation.verticalOrientation == ZONE_COMMAND_SELECTION) {
				  handleWahWahEvents(WAHWAH_EVENT_ENTERED_CMD_SLCT_MODE, 0, 0, orientation);
				  selectedCommand = 0;
				  state = CMD_SELECTION_MODE;
				  break;
			  }
		  }
		  break;
		}
	}

	if (state == CMD_SELECTION_MODE && eventId == TimerTickMngt::TIMER_TICK_100MS && false) {
		Serial.print(orientation.angleAboutVertical.distance_to_mid_scaler);
		Serial.print(" ");
		Serial.print(orientation.angleAboutVertical.in_step_value);
		Serial.print(" ");
		Serial.println(orientation.angleAboutVertical.calculateDistanceToMid());
	}
		
	handleWahWahEvents(eventId, srcRef, eventInfo, orientation);
}

void setAllLights(uint8_t colour, uint8_t saturation, uint8_t brightness) {

	for (int i = 0; i < myPie.num_lights; i++) {
		wahWahLights[i].setLight(colour, saturation, brightness);
	}
}

// This responds to events from the WahWah "Model" and updates the BlueberryPie
// in response to WAHWAH events
void handleWahWahEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, Orientation orientation) {
	const uint8_t SAT_HIGH = 255;
	const uint8_t SAT_LOW = 210;
	const uint8_t BRIGHT_HIGH = 64;
	const uint8_t BRIGHT_MED = 32;
	const uint8_t BRIGHT_LOW = 16;

	const uint8_t cmdColours[NUM_MIDI_CMDS] = { 0, 85, 170, 42, 230 };
	const uint8_t midiNote[NUM_MIDI_CMDS] = { MIDI_A2, MIDI_B2, MIDI_C3, MIDI_D3, MIDI_E3 };

	switch (eventId) {
	case WAHWAH_EVENT_TURN_ON: {
		Serial.println("ON!!");
		static SigGen::SAMPLE colourWave[] = { 255, 200 };
		setAllLights(200, SAT_HIGH, BRIGHT_LOW);
		for (int i = 0; i < myPie.num_lights; i++) {
			wahWahLights[i].brightnessGen.configSamples(sineWave255, NUM_SAM(sineWave255), 300, 0);
			wahWahLights[i].brightnessGen.start(2, i * 50);

			wahWahLights[i].colourGen.configSamples(colourWave, NUM_SAM(colourWave), 800, 0);
			wahWahLights[i].colourGen.start(1);
		}
	}
    break;

	case WAHWAH_EVENT_TURN_OFF: {
		Serial.println("OFF!!");
		static SigGen::SAMPLE turnOffSig[] = { 0xff, 0 };
		setAllLights(220, 0, 0x00);
		for (int i = 0; i < myPie.num_lights; i++) {
			wahWahLights[i].brightnessGen.configSamples(turnOffSig, NUM_SAM(turnOffSig), 300, 0);
			wahWahLights[i].brightnessGen.start(1);
		}
	}
    break;

	case WAHWAH_EVENT_ENTERED_CMD_SLCT_MODE: {
		Serial.println("ENTERED CMD SLCT MODE!!");
		setAllLights(200, SAT_HIGH, BRIGHT_MED);
		static SigGen::SAMPLE brightWav[] = { BRIGHT_LOW, 0xff, BRIGHT_MED };
		for (int i = 0; i < myPie.num_lights; i++) {
			wahWahLights[i].brightnessGen.configSamples(brightWav, NUM_SAM(brightWav), 300, 0);
			wahWahLights[i].brightnessGen.start(1);
		}
	}
    break;

	case WAHWAH_EVENT_LEFT_CMD_SLCT_MODE: {
		Serial.println("LEFT CMD SLCT MODE!!");
		setAllLights(200, SAT_LOW, BRIGHT_MED);
		static SigGen::SAMPLE brightWav[] = { BRIGHT_MED, 0xff, BRIGHT_LOW };
		for (int i = 0; i < myPie.num_lights; i++) {
			wahWahLights[i].brightnessGen.configSamples(brightWav, NUM_SAM(brightWav), 300, 0);
			wahWahLights[i].brightnessGen.start(1);
		}
	}
    break;

	case WAHWAH_EVENT_CMD_SLCT_NEW: {
		Serial.print("New Cmd: ");
		Serial.println(eventInfo);
		if (pedalOn[eventInfo]) {
			setAllLights(cmdColours[eventInfo], SAT_HIGH, 0);
		}
		else {
			setAllLights(cmdColours[eventInfo], SAT_LOW, 0);
		}
		for (int i = 0; i < myPie.num_lights; i++) {
			wahWahLights[i].brightnessTracker = &currentOrientation.angleAboutVertical.distance_to_mid_255;
		}

	}
    break;

	case WAHWAH_EVENT_CMD_SLCT_NULL: {
		Serial.println("New Null: ");
		setAllLights(200, SAT_HIGH, BRIGHT_MED);
	}
    break;

	case WAHWAH_EVENT_CMD_TOGGLE: {
		if (pedalOn[eventInfo]) {
			// Turn Off
			Serial.print("ENTERED CMD TOGGLE OFF:");
			Serial.println(eventInfo);
			static SigGen::SAMPLE brightnessWave[] = { BRIGHT_HIGH, 0xff, BRIGHT_MED };
			setAllLights(cmdColours[eventInfo], SAT_LOW, BRIGHT_MED);
			for (int i = 0; i < myPie.num_lights; i++) {
				wahWahLights[i].brightnessGen.configSamples(brightnessWave, NUM_SAM(brightnessWave), 300, 0);
				wahWahLights[i].brightnessGen.start(1);
			}
			pedalOn[eventInfo] = false;
			myPie.midiBle.noteOff(1, midiNote[eventInfo]);
		} else {
			// Turn On
			Serial.print("ENTERED CMD TOGGLE ON:");
			Serial.println(eventInfo);
			static SigGen::SAMPLE brightnessWave[] = { BRIGHT_MED, 0xff, BRIGHT_HIGH };
			setAllLights(cmdColours[eventInfo], SAT_HIGH, BRIGHT_HIGH);
			for (int i = 0; i < myPie.num_lights; i++) {
				wahWahLights[i].brightnessGen.configSamples(brightnessWave, NUM_SAM(brightnessWave), 300, 0);
				wahWahLights[i].brightnessGen.start(1);
			}
			pedalOn[eventInfo] = true;
			myPie.midiBle.noteOn(1, midiNote[eventInfo]);
		}
	}
    break;

	case WAHWAH_EVENT_CMD_CONT_MODE_ENTERED: {
		Serial.println("ENTERED CMD CONTINUOUS MODE");
		static SigGen::SAMPLE brightnessWave[] = { BRIGHT_MED, 0xff, BRIGHT_HIGH };
		setAllLights(cmdColours[eventInfo], SAT_HIGH, BRIGHT_HIGH);
		for (int i = 0; i < myPie.num_lights; i++) {
			wahWahLights[i].brightnessGen.configSamples(brightnessWave, NUM_SAM(brightnessWave), 300, 0);
			wahWahLights[i].brightnessGen.start(1);
		}
		pedalOn[eventInfo] = true;
		myPie.midiBle.noteOn(1, midiNote[eventInfo]);
	}
    break;
	}
}


// handleSerialInput
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

		case 'm': {
			uint8_t status = Serial.parseInt();
			uint8_t data1 = Serial.parseInt();
			uint8_t data2 = Serial.parseInt();
			Serial.println("Sending Midi Message");
			myPie.midiBle.setMidiMsg(status, data1, data2);
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

		case 'r': {
			// Changes offset so facing 'null' horizontal direction
			Serial.println("Reset Horizontal Orientation");
			currentOrientation.angleAboutVertical.stop(); // this avoids a burst of events as the angle about Z suddenly changes
			myPie.motionSensor.resetHorizontalOrientation();
		}
		break;

		}
	}
}

void printOrientation() {
	Serial.print("Y: ");
	Serial.print(currentOrientation.angleAboutVertical.current_continuous_value);
	Serial.print(" Y: ");
	Serial.print(currentOrientation.angleAboutVertical.current_discrete_value);
	Serial.print(" P: ");
	Serial.print(currentOrientation.angleToVertical.current_continuous_value);
	Serial.print(" P: ");
	Serial.print(currentOrientation.angleToVertical.current_discrete_value);
	Serial.print(" R: ");
	Serial.print(currentOrientation.rotationAboutNeck.current_continuous_value);
	Serial.print(" R: ");
	Serial.println(currentOrientation.rotationAboutNeck.current_discrete_value);
}

