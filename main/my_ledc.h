#ifndef MY_LEDC_H
#define MY_LEDC_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include "my_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MY_ESP_ERR_LEDC(x)                      MY_ESP_ERR(MY_ESP_ERR_LEDC_BASE, (x))
#define MY_ESP_ERR_LEDC_FREQUENCY_IS_TOO_HIGH   MY_ESP_ERR_LEDC(1)
#define MY_ESP_ERR_LEDC_FREQUENCY_IS_TOO_LOW    MY_ESP_ERR_LEDC(2)

esp_err_t myledc_set_registers(
      gpio_num_t        gpio_num
	, ledc_channel_t    ledc_channel
	, ledc_timer_t      timer_num
	, uint32_t			duty
	, ledc_timer_bit_t  duty_resolution
	, uint64_t			div_param
	, ledc_clk_src_t	timer_clk_src
	, ledc_mode_t		speed_mode
	, ledc_intr_type_t	intr_type
);

esp_err_t myledc_set_frequency_and_duty(
      gpio_num_t        gpio_num
    , double            freq_hz
    , double            duty_ratio
    , ledc_timer_bit_t  duty_resolution
    , ledc_timer_t      timer_num
    , ledc_channel_t    ledc_channel
);


#ifdef __cplusplus
} // extern "C"
#endif

#endif
