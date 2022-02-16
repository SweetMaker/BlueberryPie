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
	retVal = StrawberryString::init();
	if (retVal != 0) {
		return(-1);
	}

//	bleMidi.configDeviceName("MadCap");
 //   retVal = ble_setup();
	if (retVal != 0)
	{
		Serial.println("ble_setup failed");
		return(-2);
	}

	retVal = midiBle.setup();
	if (retVal != 0) {
		Serial.println("bleMidi setup failed");
		return(-3);
	}

//	retVal = bleMidi.start_adv();
	if (retVal != 0) {
		Serial.println("ble start_adv failed");
		return(-4);
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