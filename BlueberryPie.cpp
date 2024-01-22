#include <BLEDevice.h>
#include <Arduino.h>
#include "BlueberryPie.h"

using namespace SweetMaker;

BlueberryPie::BlueberryPie()
{
}

int BlueberryPie::init()
{
	int retVal;
	Serial.println("BlueberryPie::init");
	retVal = StrawberryString::init();
	if (retVal != 0) {
		return(-1);
	}

	Serial.println("About to setup midiBle");

	retVal = midiBle.setup();
	if (retVal != 0) {
		Serial.println("midiBle setup failed");
		return(-3);
	}

	Serial.println("Blueberry init complete");
	return (0);
}


static void __attribute__((unused)) remove_all_bonded_devices(void)
{
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t* dev_list = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	for (int i = 0; i < dev_num; i++) {
		esp_ble_remove_bond_device(dev_list[i].bd_addr);
	}

	free(dev_list);
}