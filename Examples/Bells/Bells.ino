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

struct MusicalScale {
	const uint8_t* midi_notes;
	const uint8_t num_notes;
};

struct NoteTuple {
	uint8_t midiNote;
	uint8_t indexInScale;
};

uint8_t static const hue[] = { 0,80,160,200,40,230,120 };

BlueberryPie myPie;

ToDiscrete zAxisRotationQuantizer(2048, 200); // Used for accidental selection
ToDiscrete verticalTiltQuantizer(4096, 200);  // Used for controlling playing notes
ToDiscrete rotationAboutVerticalQuantizer(4096, 512);  // Used for selecting notes
int16_t zAxisAboutVertical_16384;
int16_t verticalVelocity;

SigGen mySigGen;
SigGen sineWave(sineWave255, NUM_SAM(sineWave255), 500, 0);;
StaticGen myStaticGen;
static const SigGen::SAMPLE PROGMEM flashLowWave[] = { 127, 0, 0, 0, 0, 127 };
static const SigGen::SAMPLE PROGMEM flashHighWave[] = { 127, 200, 200, 200, 200, 127 };

int8_t saturation = 255;
ColourHSV lightStripHSV[NUM_LIGHTS];

#define MIDI_CHAN_NUM (1)

boolean performPitchBend(int16_t input, uint8_t channel);
boolean performAfterTouch(int16_t input, uint8_t channel, uint8_t note);
boolean performModulation(int16_t input, uint8_t channel);

int8_t  detectAccidental();
struct NoteTuple selectMidiNoteFromScale(MusicalScale* scale, int16_t index);

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
	if (retVal != 0) {
		Serial.println("myPie init failure");
	}

	// Setup some starting values for the LEDs
	lightStripHSV[0].setColour(0, 255, 127);
	lightStripHSV[1].setColour(160, 255, 127);
	lightStripHSV[2].setColour(80, 255, 127);
	lightStripHSV[3].setColour(200, 255, 127);
	lightStripHSV[4].setColour(120, 255, 127);
}

/*
 * Called repeatedly after setup. 
 */
void loop()
{
	// Update each light from HSV value to RGB - Brightness is set depending on mySigGen
	uint8_t value = mySigGen.isRunning() ? (uint8_t)mySigGen.readValue() : 127; // Used for Lights HSV value (brightness)
	uint8_t scaledSaturation = saturation;
	if (sineWave.isRunning())
		scaledSaturation = ((uint8_t)sineWave.readValue() * saturation) >> 8;
	
	for (uint8_t i = 0; i < NUM_LIGHTS; i++) {
		ColourHSV& hsv = lightStripHSV[i];
		myPie.ledStrip[i] = ColourConverter::ConvertToRGB(hsv.hue, scaledSaturation, value);
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
	rotationAboutVerticalQuantizer.writeValue((int32_t)zAxisAboutVertical_16384);

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
 * Select Midi Note from scale - this handles shifting octaves if the index is large
 */
//const uint8_t c_major_notes[] = { MIDI_C4,MIDI_D4,MIDI_E4,MIDI_F4,MIDI_G4,MIDI_A4,MIDI_B4 };
//MusicalScale c_major_scale = { c_major_notes, 7 };

const uint8_t e_major_notes[] = { MIDI_CS3, MIDI_DS3, MIDI_E3,MIDI_FS3,MIDI_GS3, MIDI_A3,MIDI_B3 };
MusicalScale e_major_scale = { e_major_notes, 7 };

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
	STRIKE_ZONE = 3, // Used for controlling the playing of notes
}BELLS_ZONE;

/* This maps the quantized tilt to 'zones' used by Bells */
uint8_t convertTiltToZone(int16_t tilt) {
	static int16_t zoneMap[] = { 
		MUTE_ZONE,
		DAMP_ZONE,
		NOTE_SELECTION_ZONE,
		NOTE_SELECTION_ZONE,
		STRIKE_ZONE
	};
	if (tilt < 0) tilt = 0;
	if (tilt > 4) tilt = 4;
	return (zoneMap[tilt]);
}

void BELLS_handle_event() {
	/*
	* This is the state information we use
	*/
	static BELLS_STATE state;
	static struct NoteTuple nextNote = { 0,0 };
	static uint8_t currentNote = 0;
	static int16_t lastOrientation = 0;
	static int16_t maxVerticalVelocity = 0;
	static int16_t pitchBendNullPosition = 0;
	static boolean dampPedalOn = true;

	int16_t tilt = verticalTiltQuantizer.current_discrete_value;
	uint8_t zone = convertTiltToZone(tilt);

	switch (state) {
	case NOTE_SELECTION: {
		if (zone == STRIKE_ZONE) {
			Serial.print("Has started strike: ");
			Serial.println(nextNote.midiNote);
			state = PRE_STRIKE;
			mySigGen.configSamples(flashHighWave, NUM_SAM(flashHighWave), 200, SigGen::DONT_FINISH_ON_ZERO);
			mySigGen.start(1);
			maxVerticalVelocity = 0;
			saturation = 0;
			break;
		}
		if ((zone == DAMP_ZONE) || (zone == MUTE_ZONE)) {
			Serial.println("Has started damping");
			state = DAMPING;
			break;
		}
		int16_t orientation = rotationAboutVerticalQuantizer.current_discrete_value;

		nextNote = selectMidiNoteFromScale(&e_major_scale, orientation);
		nextNote.midiNote += detectAccidental();
		uint8_t sat = 255;
		if ((zAxisRotationQuantizer.in_step_value < 0) || (zAxisRotationQuantizer.in_step_value > 3000))
			sat = 168;
		for (uint8_t i = 0; i < NUM_LIGHTS; i++)
			lightStripHSV[i].setColour(hue[nextNote.indexInScale], sat, 127);
	}
    break;

	case PRE_STRIKE: {
		if (verticalVelocity > maxVerticalVelocity) {
			maxVerticalVelocity = verticalVelocity;
		}

		if (verticalVelocity < 0) {
			Serial.print("Has struck: ");
			uint8_t velocity = (uint8_t)((uint16_t)maxVerticalVelocity >> 3);
			Serial.println(velocity);
			if (velocity > 127) velocity = 127;

			myPie.midiBle.noteOff(MIDI_CHAN_NUM, currentNote);
			myPie.midiBle.noteOn(MIDI_CHAN_NUM, nextNote.midiNote, velocity);
			currentNote = nextNote.midiNote;

			mySigGen.configSamples(flashLowWave, NUM_SAM(flashLowWave), 200, SigGen::DONT_FINISH_ON_ZERO);
			mySigGen.start(1);

			pitchBendNullPosition = zAxisAboutVertical_16384;
			saturation = 128 + velocity;
			sineWave.start();
			state = POST_STRIKE;
		}
	}
	break;
				    
	case POST_STRIKE: {
		if (zone == NOTE_SELECTION_ZONE) {
			Serial.println("Has started note selection");
			myPie.midiBle.noteOff(MIDI_CHAN_NUM, currentNote);
			saturation = 255;
			sineWave.stop();
			state = NOTE_SELECTION;
			break;
		}
		// Bend note
		//performPitchBend(zAxisAboutVertical_16384 - pitchBendNullPosition, MIDI_CHAN_NUM);
		//performAfterTouch(zAxisAboutVertical_16384 - pitchBendNullPosition, MIDI_CHAN_NUM, currentNote);
		performModulation(zAxisAboutVertical_16384 - pitchBendNullPosition, MIDI_CHAN_NUM);
		}
    break;

	case DAMPING: {
		if (zone == DAMP_ZONE) {
			uint8_t footControlValue = (uint8_t)(((uint16_t)verticalTiltQuantizer.in_step_value) >> 5);
			myPie.midiBle.setMidiMsg(0b10110000, 4, footControlValue);
		}
		else if (zone == MUTE_ZONE) {
			if (dampPedalOn) {
				Serial.println("Sustain pedal off");
				dampPedalOn = false;
				myPie.midiBle.damperPedalOff(MIDI_CHAN_NUM);
				lightStripHSV[0].hue = 87;
			}
			else {
				Serial.println("Sustain pedal on");
				dampPedalOn = true;
				myPie.midiBle.damperPedalOn(MIDI_CHAN_NUM);
				lightStripHSV[0].hue = 0;
			}
			state = MUTE;
		}
		else {
			Serial.println("Has started note selection");
			myPie.midiBle.setMidiMsg(0b10110000, 4, 127);
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

boolean performPitchBend(int16_t input, uint8_t channel) {
	if (input < 2048)
		return false;

	uint16_t bendVal = (input + 2048) << 1;
	if (bendVal > 0x3fff)
		bendVal = 0x3fff;

	myPie.midiBle.pitchBendChange(MIDI_CHAN_NUM, bendVal);
	return true;
}

boolean performAfterTouch(int16_t input, uint8_t channel, uint8_t note) {

	if (input > -2048)
		return false;
	uint16_t bendVal = (-input - 2048) << 1;
	if (bendVal > 0x1fff)
		bendVal = 0x1fff;

	myPie.midiBle.polyphonicPressure(channel, note, bendVal >> 6);
	return true;
}

boolean performModulation(int16_t input, uint8_t channel) {

	if (input > -2048)
		return false;
	uint16_t modVal = (-input - 2048) << 1;
	if (modVal > 0x3fff)
		modVal = 0x3fff;

	myPie.midiBle.modulate(channel, modVal);
	return true;
}


int8_t  detectAccidental() {
	if (zAxisRotationQuantizer.current_discrete_value >= 2)
		return -1;
	else if (zAxisRotationQuantizer.current_discrete_value <= -2)
		return +1;
	return 0;
}


/*
 * selectMidiNoteFromScale
 */
struct NoteTuple selectMidiNoteFromScale(MusicalScale* scale, int16_t index) {
	int8_t octave_shift = 0;
	while (index < 0) {
		index += scale->num_notes;
		octave_shift--;
	}
	while (index >= scale->num_notes) {
		index -= scale->num_notes;
		octave_shift++;
	}
	int16_t midi_note = scale->midi_notes[index];
	return { (uint8_t)(midi_note + octave_shift * 12), (uint8_t)index };
}
