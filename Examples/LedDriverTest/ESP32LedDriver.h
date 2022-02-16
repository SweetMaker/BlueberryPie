
#ifndef __ESP32_LED_DRIVER_H__
#define __ESP32_LED_DRIVER_H__

#include <stdint.h>
#include "driver/rmt.h"
#include "Colour.h"

namespace SweetMaker
{
	class Esp32LedDriver
	{
	public:
		Esp32LedDriver();
		~Esp32LedDriver();
		int config(uint8_t pinNum, uint16_t numLeds);
		int txLedString(ColourRGB * ledString, uint16_t numLeds);

	private:
		rmt_item32_t item_high, item_low, item_show;
		rmt_item32_t * itemArray;
		uint16_t numItems;
		rmt_channel_t rmtChannelNum;

		int configRmtItems(uint16_t numLeds);
		int selectRmtChannel();
		int configRmtChannel(rmt_channel_t channelNum, uint8_t pinNum);

		int byte2Items(rmt_item32_t * itemBuf, uint16_t numItems, uint8_t byte);
		int rgb2Items(rmt_item32_t * itemBuf, uint16_t numItems, ColourRGB * rgb);
		int ledString2Items(rmt_item32_t * itemBuf, uint16_t numItems, ColourRGB * rgb, uint16_t numLeds);
	};
}

#endif