#include <string.h>
#include <array>
#include "my_rmt.h"

namespace {
const int		RmtItem32Max	= 64;
const int		RmtItem16Max	= RmtItem32Max * 2;
const int		RmtDurationMax	= 32767;
}

static rmt_item16_t* myrmt_writeItem(rmt_item16_t* pItem, rmt_item16_t* pItemEnd, int32_t duration, int level) {
	auto d = duration;
	const int l = level ? 1 : 0;
	while(d > 0 && pItem < pItemEnd) {
		auto t = d;
		if(t > RmtDurationMax) {
			t = RmtDurationMax;
		}
		pItem->duration = t;
		pItem->level = l;

		++pItem;
		d -= t;
	}
	return pItem;
}


static rmt_item16_t* myrmt_writePadding(rmt_item16_t* pItem, rmt_item16_t* pItemTop, rmt_item16_t* pItemEnd) {
	const int nItem = static_cast<int>(pItem - pItemTop);
	if(nItem & 1) {
		if(pItem < pItemEnd) {
			pItem->duration = 0;
			pItem->level = 0;
			++pItem;
		}
	}
	return pItem;
}


esp_err_t myrmt_setup_pulse_output(
	  rmt_channel_t		channel
	, gpio_num_t		outputGpioNum
	, int32_t			clkDiv
	, int32_t			durationA
	, int32_t			levelA
	, int32_t			durationB
	, int32_t			levelB
	, int32_t*			pNumItems
) {
	std::array<rmt_item32_t, RmtItem32Max> items;
	memset(items.data(), 0, items.size() * sizeof(items[0]));

	auto* const pItemTop = reinterpret_cast<rmt_item16_t*>(items.data());
	auto* const pItemEnd = reinterpret_cast<rmt_item16_t*>(std::end(items));

	auto* p = pItemTop;

	p = myrmt_writeItem(p, pItemEnd, durationA, levelA);
	p = myrmt_writeItem(p, pItemEnd, durationB, levelB);
	p = myrmt_writePadding(p, pItemTop, pItemEnd);

	const auto nItems = static_cast<uint32_t>(p - pItemTop) / 2;
	if(pNumItems != nullptr) {
		*pNumItems = nItems;
	}

	rmt_config_t c;
	memset(&c, 0, sizeof(c));
	auto& tx = c.tx_config;
	c.rmt_mode				= RMT_MODE_TX;
	c.channel				= channel;
	c.gpio_num				= outputGpioNum;
	c.mem_block_num			= nItems;
	c.clk_div				= clkDiv;

	tx.loop_en				= 1;

	tx.idle_output_en		= 1;
	tx.idle_level			= RMT_IDLE_LEVEL_LOW;

	tx.carrier_en			= 0;
	tx.carrier_freq_hz		= 1;
	tx.carrier_level		= RMT_CARRIER_LEVEL_LOW;
	tx.carrier_duty_percent	= 50;

	if(nItems > 16) {
		return MY_ESP_ERR_RMT_ITEM_BUFFER_OVERFLOW;
	}
	ESP_ERROR_CHECK(rmt_driver_install(channel, 0, ESP_INTR_FLAG_SHARED));
	ESP_ERROR_CHECK(rmt_config(&c));
	ESP_ERROR_CHECK(rmt_write_items(channel, reinterpret_cast<rmt_item32_t*>(pItemTop), nItems, false));
	rmt_tx_stop(channel);
	return ESP_OK;
}
