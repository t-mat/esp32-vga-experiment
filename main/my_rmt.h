#ifndef MY_RMT_H
#define MY_RMT_H

#include <driver/rmt.h>
#include "my_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MY_ESP_ERR_RMT(x)       				MY_ESP_ERR(MY_ESP_ERR_RMT_BASE, (x))
#define MY_ESP_ERR_RMT_ITEM_BUFFER_OVERFLOW		MY_ESP_ERR_RMT(1)

esp_err_t myrmt_setup_pulse_output(
	  rmt_channel_t		channel
	, gpio_num_t		outputGpioNum
	, int32_t			clkDiv
	, int32_t			durationA
	, int32_t			levelA
	, int32_t			durationB
	, int32_t			levelB
	, int32_t*			numItems
);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
