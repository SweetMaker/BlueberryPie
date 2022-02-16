#include <Arduino.h>
#include <BLE2902.h>

#include "EventMngr.h"
#include "MidiBle.h"

using namespace SweetMaker;

#define SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

MidiBle::MidiBle()
{
}

void MidiBle::setup()
{
	BLEDevice::init("SweetMaker");

	pServer = BLEDevice::createServer();
	pServer->setCallbacks(&serverCallbacks);

	pService = pServer->createService(SERVICE_UUID);
	pCharacteristic = pService->createCharacteristic(
		BLEUUID(CHARACTERISTIC_UUID),
		BLECharacteristic::PROPERTY_READ |
		BLECharacteristic::PROPERTY_WRITE |
		BLECharacteristic::PROPERTY_NOTIFY |
		BLECharacteristic::PROPERTY_WRITE_NR
	);

	pCharacteristic->addDescriptor(new BLE2902());

	pCharacteristic->setValue("BLE-MIDI 1.0");

	pService->start();

	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
	pAdvertising->setMinPreferred(0x12);
	pAdvertising->start();

//	BLEDevice::startAdvertising();
}

void MidiBle::setMidiMsg(uint8_t status)
{
	encodeMidiMsg(millis(), status);
	pCharacteristic->setValue(encodedBuf, 3); // packet, length in bytes
	pCharacteristic->notify();
}

void MidiBle::setMidiMsg(uint8_t status, uint8_t data1)
{
	encodeMidiMsg(millis(), status, data1);
	pCharacteristic->setValue(encodedBuf, 4); // packet, length in bytes
	pCharacteristic->notify();
}

void MidiBle::setMidiMsg(uint8_t status, uint8_t data1, uint8_t data2)
{
	encodeMidiMsg(millis(), status, data1, data2);
	pCharacteristic->setValue(encodedBuf, 5); // packet, length in bytes
	pCharacteristic->notify();
}

void MidiBle::encodeMidiMsg(uint16_t timestamp_ms, uint8_t status)
{
	memset(encodedBuf, 0, sizeof(encodedBuf));
	encodedBuf[0] = (timestamp_ms >> 7) & 0x3f;
	encodedBuf[0] |= 0x80;

	encodedBuf[1] = timestamp_ms & 0x7f;
	encodedBuf[1] |= 0x80;

	encodedBuf[2] = status;
	encodedBuf[2] |= 0x80;

	encodedLen = 3;
}

void MidiBle::encodeMidiMsg(uint16_t timestamp_ms, uint8_t status, uint8_t data1)
{
	encodeMidiMsg(timestamp_ms, status);
	encodedBuf[3] = data1;
	encodedBuf[3] &= 0x7f;

	encodedLen = 4;
}

void MidiBle::encodeMidiMsg(uint16_t timestamp_ms, uint8_t status, uint8_t data1, uint8_t data2)
{
	encodeMidiMsg(timestamp_ms, status, data1);
	encodedBuf[4] = data2;
	encodedBuf[4] &= 0x7f;

	encodedLen = 5;
}

void MidiBle::ServerCallbacks::onConnect(BLEServer * pServer)
{
	EventMngr::getMngr()->handleEvent(BME_CONNECT, 0, 0);
}

void MidiBle::ServerCallbacks::onDisconnect(BLEServer * pServer)
{
	EventMngr::getMngr()->handleEvent(BME_DISCONNECT, 0, 0);
}
