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
#define WAHWAH_EVENT_CMD_TOGGLE_MODE_ENTERED	(IEventHandler::USER + 6)
#define WAHWAH_EVENT_CMD_CONT_MODE_ENTERED			(IEventHandler::USER + 7)

// These are Identifiers for the ToDiscrete instances so we can differentiate events from each
#define ANGLE_ABOUT_VERTICAL_REF	(0)
#define ANGLE_TO_VERTICAL_REF		(1)
#define ROTATION_ABOUT_NECK_REF		(2)

// These are used to help configure the ToDiscrete instances
#define NUM_VERTICAL_SEGMENTS       (16)
#define NUM_HORIZONTAL_SEGMENTS     (16)
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
		Serial.print(currentOrientation.verticalOrientation);
		Serial.print(" ");
		printOrientation();
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

		currentOrientation.verticalOrientation = (SpecificOrientation) verticalZoneMap[currentOrientation.angleToVertical.current_discrete_value];
		currentOrientation.neckRotationOrientation = ZONE_NON_SPECIFIC;
		if ((currentOrientation.rotationAboutNeck.current_discrete_value > 1) || (currentOrientation.rotationAboutNeck.current_discrete_value < -2))
			currentOrientation.neckRotationOrientation = TILTED_ABOUT_NECK;

		// printOrientation();
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

	if(eventId == MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY) {
		/* Start by managing ON and OFF state */
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

		/* Priniciple state machine */
		switch(state) {
          case ON:
			if (orientation.verticalOrientation == ZONE_COMMAND_SELECTION) {
				myPie.motionSensor.resetHorizontalOrientation();
				currentOrientation.angleAboutVertical.start(0);
				state = CMD_SELECTION_MODE;
				handleWahWahEvents(WAHWAH_EVENT_ENTERED_CMD_SLCT_MODE, 0, 0, orientation);
			}
			break;

		  case CMD_SELECTION_MODE: {
			  if (orientation.verticalOrientation == ZONE_COMMAND_SELECTION)
				  /* nothing to do*/
				  break;

			  int16_t selectedCommand = -1 - orientation.angleAboutVertical.current_discrete_value;
			  if ((selectedCommand < 0) || (selectedCommand >= NUM_MIDI_CMDS)) {
				  handleWahWahEvents(WAHWAH_EVENT_LEFT_CMD_SLCT_MODE, 0, 0, orientation);
				  state = ON;
				  break;
			  }

			  if (orientation.verticalOrientation == ZONE_TOGGLE) {
				  state = CMD_TOGGLE_MODE;
				  handleWahWahEvents(WAHWAH_EVENT_CMD_TOGGLE_MODE_ENTERED, 0, selectedCommand, orientation);
				  break;
			  }
			  if (orientation.verticalOrientation == ZONE_CONTINUOUS_CONTROL) {
				  state = CMD_CONTINOUS_CONTROL_MODE;
				  handleWahWahEvents(WAHWAH_EVENT_CMD_CONT_MODE_ENTERED, 0, 0, orientation);
				  break;
			  }
			  state = ON;
			  handleWahWahEvents(WAHWAH_EVENT_LEFT_CMD_SLCT_MODE, 0, 0, orientation);
		  }
		  break;
		}
	}

	if (state == CMD_SELECTION_MODE && eventId == ToDiscrete::NEW_VALUE && srcRef == ANGLE_ABOUT_VERTICAL_REF) {
		handleWahWahEvents(WAHWAH_EVENT_CMD_SLCT_NEW, 0, orientation.angleAboutVertical.current_discrete_value, orientation);
	}
		
	handleWahWahEvents(eventId, srcRef, eventInfo, orientation);
}

// This responds to events from the WahWah "Model" and updates the BlueberryPie
// in response to WAHWAH events
void handleWahWahEvents(uint16_t eventId, uint8_t srcRef, uint16_t eventInfo, Orientation orientation){
	if (eventId == WAHWAH_EVENT_TURN_ON)
		Serial.println("ON!!");
	if (eventId == WAHWAH_EVENT_TURN_OFF)
		Serial.println("OFF!!");
	if (eventId == WAHWAH_EVENT_ENTERED_CMD_SLCT_MODE)
		Serial.println("ENTERED CMD SLCT MODE!!");
	if (eventId == WAHWAH_EVENT_LEFT_CMD_SLCT_MODE)
		Serial.println("LEFT CMD SLCT MODE!!");
	if (eventId == WAHWAH_EVENT_CMD_SLCT_NEW) {
		Serial.print("New Cmd: ");
		Serial.print(orientation.angleAboutVertical.current_continuous_value);
		Serial.print(": ");
		Serial.println(orientation.angleAboutVertical.current_discrete_value);
	}
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
