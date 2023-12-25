#ifndef __MIDI_BLE_H__
#define __MIDI_BLE_H__

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "IEventHandler.h"

namespace SweetMaker 
{
	class MidiBle {
	public :
		MidiBle();

		int setup(void);
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
