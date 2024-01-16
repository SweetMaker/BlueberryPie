#include <Arduino.h>
#include <BLE2902.h>

#include "EventMngr.h"
#include "MidiBle.h"

using namespace SweetMaker;

#define SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

float blePairTimeOut = 0;

class MySecurity : public BLESecurityCallbacks {

	bool onConfirmPIN(uint32_t pin) {
    Serial.print("onConfirmPIN"); Serial.print(" "); Serial.println(blePairTimeOut);
    if (blePairTimeOut >= 0) {
			return true;
		}
		else {
			return false;
		}
	}

	uint32_t onPassKeyRequest() {
		Serial.println("onPassKeyRequest");
		ESP_LOGI(LOG_TAG, "PassKeyRequest");
		return 133700;
	}

	void onPassKeyNotify(uint32_t pass_key) {
		Serial.println("onPassKeyNotify");
		ESP_LOGI(LOG_TAG, "On passkey Notify number:%d", pass_key);
	}

	bool onSecurityRequest() {
		Serial.println("onSecurityRequest");
		ESP_LOGI(LOG_TAG, "On Security Request");
		return true;
	}

	void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
		ESP_LOGI(LOG_TAG, "Starting BLE work!");
		if (cmpl.success) {
			Serial.println("onAuthenticationComplete -> success");
			uint16_t length;
			esp_ble_gap_get_whitelist_size(&length);
			ESP_LOGD(LOG_TAG, "size: %d", length);
		}
		else {
      Serial.println(cmpl.auth_mode);
      Serial.println(cmpl.fail_reason);
      Serial.println("onAuthenticationComplete -> fail");
		}
	}
};

#define MIDI_VELOCITY_DEFAULT		(60)

#define MIDI_NOTE_OFF				(0x80)
#define MIDI_NOTE_ON				(0x90)
#define MIDI_POLYPHONIC_PRESSURE	(0xA0)
#define MIDI_CHAN_CONTROL			(0xb0)
#define MIDI_PITCH_BEND				(0xE0)

#define MIDI_CC_MODULATION          (1)
#define MIDI_CC_DAMPER_PEDAL        (64)

MidiBle::MidiBle()
{
}

int MidiBle::setup()
{
	BLEDevice::init("SweetMaker");
	BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
	BLEDevice::setSecurityCallbacks(new MySecurity());


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

	BLESecurity* pSecurity = new BLESecurity();
	pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND); //ESP_LE_AUTH_REQ_SC_ONLY
	pSecurity->setCapability(ESP_IO_CAP_KBDISP);
	pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    Serial.println("MidiBle: Starting to advertise");
	BLEDevice::startAdvertising();
	return(0);
}


void MidiBle::noteOn(uint8_t channel, uint8_t note, uint8_t velocity){
	setMidiMsg(MIDI_NOTE_ON | (channel-1), note, velocity);
}

void MidiBle::noteOn(uint8_t channel, uint8_t note){
	noteOn(channel, note, MIDI_VELOCITY_DEFAULT);
}

void MidiBle::noteOff(uint8_t channel, uint8_t note, uint8_t velocity){
	setMidiMsg(MIDI_NOTE_OFF | (channel-1), note, velocity);
}

void MidiBle::noteOff(uint8_t channel, uint8_t note){
	noteOff(channel, note, MIDI_VELOCITY_DEFAULT);
}

void MidiBle::pitchBendChange(uint8_t channel, uint16_t bender_16384) {
	setMidiMsg(MIDI_PITCH_BEND | (channel-1), (uint8_t)bender_16384 | 0x3f, (uint8_t)(bender_16384 >> 7));
}

void MidiBle::polyphonicPressure(uint8_t channel, uint8_t note, uint8_t pressure) {
	setMidiMsg(MIDI_POLYPHONIC_PRESSURE | (channel - 1), note, pressure);
}

void MidiBle::modulate(uint8_t channel, uint16_t value_16384) {
	setMidiMsg(MIDI_CHAN_CONTROL | (channel - 1), MIDI_CC_MODULATION, value_16384 >>7);
}


void MidiBle::damperPedalOn(uint8_t channel) {
	setMidiMsg(MIDI_CHAN_CONTROL | (channel-1), MIDI_CC_DAMPER_PEDAL, 127);
}

void MidiBle::damperPedalOff(uint8_t channel) {
	setMidiMsg(MIDI_CHAN_CONTROL | (channel-1), MIDI_CC_DAMPER_PEDAL, 0);
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
