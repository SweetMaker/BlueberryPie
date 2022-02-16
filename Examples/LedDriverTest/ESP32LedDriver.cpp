
#include <string.h>
#include <Arduino.h>
#include "ESP32LedDriver.h"
#include "driver/gpio.h"

using namespace SweetMaker;

#define F_APB_CLOCK_HZ		(80000000L)
#define APB_CLOCK_PERIOD_NS ( 1000000000L / F_APB_CLOCK_HZ )
#define RMT_DIVIDER			(2)
#define RMT_CLOCK_PERIOD_NS (APB_CLOCK_PERIOD_NS * RMT_DIVIDER)

static uint8_t activeChannelBitMap = 0;


Esp32LedDriver::Esp32LedDriver()
{

}

SweetMaker::Esp32LedDriver::~Esp32LedDriver()
{
	activeChannelBitMap &= ~(uint8_t)rmtChannelNum;
}

int Esp32LedDriver::config(uint8_t pinNum, uint16_t numLeds)
{
	if (selectRmtChannel())
		return (-1);

	configRmtItems(numLeds);
	configRmtChannel(rmtChannelNum, pinNum);
	return(0);
}




int Esp32LedDriver::configRmtItems(uint16_t numLeds)
{
#define WS2812B_CLOCK_PERIOD_NS (208)
#define T1H_ns (WS2812B_CLOCK_PERIOD_NS * 3 + 1)

	uint16_t T1L_ns = WS2812B_CLOCK_PERIOD_NS * 2 + 1;
	uint16_t T0H_ns = WS2812B_CLOCK_PERIOD_NS + 1;
	uint16_t T0L_ns = WS2812B_CLOCK_PERIOD_NS * 3 + 1;
	uint16_t RESET_ns = 6000;

	#define BITS_IN_BYTE (8)
    #define BYTES_IN_LED (3)

	this->numItems = (numLeds * BITS_IN_BYTE * BYTES_IN_LED) + 1;
		
	itemArray = (rmt_item32_t*) malloc(sizeof(rmt_item32_t) * numItems);
	if (itemArray == NULL)
		return (-1);

	item_high.level0 = 1;
	item_high.duration0 = (T1H_ns / RMT_CLOCK_PERIOD_NS) + 1;
	item_high.level1 = 0;
	item_high.duration1 = (T1L_ns / RMT_CLOCK_PERIOD_NS) + 1;

	item_low.level0 = 1;
	item_low.duration0 = (T0H_ns / RMT_CLOCK_PERIOD_NS) + 1;
	item_low.level1 = 0;
	item_low.duration1 = (T0L_ns / RMT_CLOCK_PERIOD_NS) + 1;

	item_show.level0 = 0;
	item_show.level1 = 0;
	item_show.duration0 = 0;
	item_show.duration1 = RESET_ns / RMT_CLOCK_PERIOD_NS;

	return(0);
}

int Esp32LedDriver::selectRmtChannel()
{
	uint8_t bitMask = 0x01;
	for (uint8_t channelNum = 0; channelNum < 8; channelNum++)
	{
		if (activeChannelBitMap & bitMask)	{
			bitMask = bitMask << 1;
			continue;
		}
		this->rmtChannelNum = (rmt_channel_t)channelNum;
		activeChannelBitMap |= bitMask;
		return (0);	
	}

	return (-1);
}


int Esp32LedDriver::configRmtChannel(rmt_channel_t channelNum, uint8_t pinNum)
{
	esp_err_t ret_val;
	rmt_config_t rmtConfig;
	memset(&rmtConfig, 0, sizeof(rmt_config_t));

	rmtConfig.channel = channelNum;
	rmtConfig.gpio_num = (gpio_num_t)27;
	rmtConfig.mem_block_num = 1;
	rmtConfig.rmt_mode = RMT_MODE_TX;
	rmtConfig.clk_div = RMT_DIVIDER;
	rmtConfig.tx_config.carrier_en = false;
	rmtConfig.tx_config.loop_en = false;
	rmtConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
	rmtConfig.tx_config.idle_output_en = true;

	ret_val = rmt_config(&rmtConfig);
	if (ret_val != ESP_OK)
	{
		Serial.print("rmt_config error: \n");
		Serial.println(ret_val);
	}
	else
	{
		Serial.println("rmt_config success \n");
	}

	ret_val = rmt_driver_install(this->rmtChannelNum, 0, 0);
	if (ret_val != ESP_OK)
	{
		Serial.print("rmt_driver_install error: \n");
		Serial.println(ret_val);
	}
	else
	{
		Serial.println("rmt_driver_install success \n");
	}
	return(0);
}


int Esp32LedDriver::byte2Items(rmt_item32_t * itemBuf, uint16_t numItems, uint8_t byte)
{
	if ((numItems < 8) || (itemBuf == NULL))
		return (-1);

	for (uint8_t bit_mask = 0x80; bit_mask != 0; bit_mask = bit_mask >> 1) {
		if (byte & bit_mask)
			*itemBuf++ = item_high;
		else
			*itemBuf++ = item_low;
	}
	return(0);
}


int Esp32LedDriver::rgb2Items(rmt_item32_t * itemBuf, uint16_t numItems, ColourRGB * rgb)
{
	if ((numItems < 8 * 3) || (itemBuf == NULL))
		return (-1);

	byte2Items(itemBuf, numItems, rgb->green);
	itemBuf += 8;
	numItems -= 8;

	byte2Items(itemBuf, numItems, rgb->red);
	itemBuf += 8;
	numItems -= 8;

	byte2Items(itemBuf, numItems, rgb->blue);
	return(0);
}



int Esp32LedDriver::ledString2Items(rmt_item32_t * itemBuf, uint16_t numItems, ColourRGB * rgbString, uint16_t numLeds)
{
	uint16_t requiredItems = 8 * 3 * numLeds;
	if ((numItems < requiredItems) || (itemBuf == NULL))
		return(-1);

	for (int i = 0; i < numLeds; i++)
	{
		rgb2Items(itemBuf, numItems, rgbString++);
		itemBuf += 24;
		numItems -= 24;
	}
	return(requiredItems);
}

int Esp32LedDriver::txLedString(ColourRGB * ledString, uint16_t numLeds)
{
	uint16_t itemIter;
	esp_err_t ret_val;
	memset(itemArray, 0, sizeof(rmt_item32_t)*numItems);

	itemIter = ledString2Items(itemArray, numItems, ledString, numLeds);
	if (itemIter >= numItems)
		return(-1);

	itemArray[itemIter++] = item_show;

	ret_val = rmt_write_items(rmtChannelNum, itemArray, itemIter, true);
	if (ret_val != 0)
	{
		Serial.print("rmt_write_items failed ");
		Serial.println(ret_val);
		return(-1);
	}
	return(0);
}


