// These functions are almost identical for ledc_*() functions.
// But since precision of esp-idf's ledc_*() is not enough for my application,
// I made these functions.
#include <soc/ledc_reg.h>
#include <soc/ledc_struct.h>
#include "my_ledc.h"

static esp_err_t myledc_enable_intr_type(ledc_channel_t, ledc_intr_type_t);


esp_err_t myledc_set_registers(
      gpio_num_t        gpio_num
    , ledc_channel_t    ledc_channel
    , ledc_timer_t      timer_num
    , uint32_t          duty
    , ledc_timer_bit_t  duty_resolution
    , uint64_t          div_param
    , ledc_clk_src_t    timer_clk_src
    , ledc_mode_t       speed_mode
    , ledc_intr_type_t  intr_type
) {
    if(div_param < 256) {
        return MY_ESP_ERR_LEDC_FREQUENCY_IS_TOO_HIGH;
    }
    if(div_param > 0x3ffff) {
        return MY_ESP_ERR_LEDC_FREQUENCY_IS_TOO_LOW;
    }

    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_set(speed_mode, timer_num, div_param, duty_resolution, timer_clk_src);
    ledc_timer_rst(speed_mode, timer_num);

    ledc_set_duty(speed_mode, ledc_channel, duty);
    ledc_update_duty(speed_mode, ledc_channel);
    ledc_bind_channel_timer(speed_mode, ledc_channel, timer_num);
    myledc_enable_intr_type(ledc_channel, intr_type);

    if(gpio_num >= 0) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio_num], PIN_FUNC_GPIO);
        gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
        gpio_matrix_out(gpio_num, LEDC_HS_SIG_OUT0_IDX + ledc_channel, 0, 0);
    }

    return ESP_OK;
}


esp_err_t myledc_set_frequency_and_duty(
      gpio_num_t        gpio_num
    , double            freq_hz
    , double            duty_ratio
    , ledc_timer_bit_t  duty_resolution
    , ledc_timer_t      timer_num
    , ledc_channel_t    ledc_channel
) {
    // LEDC_APB_CLK_HZ  80 000 000  (80MHz)
    // LEDC_REF_CLK_HZ   1 000 000  (1 MHz)

    const double            precision       = (1 << duty_resolution);
    const double            divApbClk       = ((double) (LEDC_APB_CLK_HZ)) * 256.0 / precision;
    const uint32_t          duty            = (uint32_t) (precision * duty_ratio);
    const uint64_t          div_param       = (uint64_t) (divApbClk / freq_hz);
    const ledc_clk_src_t    timer_clk_src   = LEDC_APB_CLK;
    const ledc_mode_t       speed_mode      = LEDC_HIGH_SPEED_MODE;
    const ledc_intr_type_t  intr_type       = LEDC_INTR_DISABLE;

    return myledc_set_registers(
          gpio_num
        , ledc_channel
        , timer_num
        , duty
        , duty_resolution
        , div_param
        , timer_clk_src
        , speed_mode
        , intr_type
    );
}


static esp_err_t myledc_enable_intr_type(
      ledc_channel_t channel
    , ledc_intr_type_t type
) {
    const uint32_t value = LEDC.int_ena.val;
    const uint32_t mask = BIT(LEDC_DUTY_CHNG_END_HSCH0_INT_ENA_S + channel);
    if (type == LEDC_INTR_FADE_END) {
        LEDC.int_ena.val = value | mask;
    } else {
        LEDC.int_ena.val = value & (~mask);
    }
    return ESP_OK;
}
