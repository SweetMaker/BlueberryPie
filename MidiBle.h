#ifndef __MIDI_BLE_H__
#define __MIDI_BLE_H__
#include <stdint.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "IEventHandler.h"
#include "midiNotes.h"

namespace SweetMaker 
{
	class MidiBle {
	public :
		MidiBle();

		int setup(void);

		void noteOn(uint8_t channel, uint8_t note, uint8_t velocity);
		void noteOn(uint8_t channel, uint8_t note);
		void noteOff(uint8_t channel, uint8_t note, uint8_t velocity);
		void noteOff(uint8_t channel, uint8_t note);
		void pitchBendChange(uint8_t channel, uint16_t bendder);
		void polyphonicPressure(uint8_t channel, uint8_t note, uint8_t pressure);
		void modulate(uint8_t channel, uint16_t value);
		void damperPedalOn(uint8_t channel);
		void damperPedalOff(uint8_t channel);
		void setVolume(uint8_t channel, uint8_t volume);
		void setFootControl(uint8_t channel, uint16_t level);


		void setMidiMsg(uint8_t status);
		void setMidiMsg(uint8_t status, uint8_t data1);
		void setMidiMsg(uint8_t status, uint8_t data1, uint8_t data2);

		enum {
			BME_CONNECT = IEventHandler::BLE_MIDI_EVENTS,
			BME_DISCONNECT = IEventHandler::BLE_MIDI_EVENTS + 1
		}BLE_MIDI_EVENTS;

	private:
		void encodeMidiMsg(uint16_t timestamp_ms, uint8_t status);
		void encodeMidiMsg(uint16_t timestamp_ms, uint8_t status, uint8_t data1);
		void encodeMidiMsg(uint16_t timestamp_ms, uint8_t status, uint8_t data1, uint8_t data2);

		uint8_t encodedBuf[128];
		uint16_t encodedLen;

		class ServerCallbacks : public BLEServerCallbacks 
		{
			void onConnect(BLEServer* pServer);
			void onDisconnect(BLEServer* pServer);
		}serverCallbacks;

		BLEServer *pServer;
		BLEService *pService;
		BLECharacteristic *pCharacteristic;
	};
}

#endif
