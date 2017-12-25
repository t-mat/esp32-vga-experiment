// These functions are almost identical for ledc_*() functions.
// But since precision of esp-idf's ledc_*() is not enough for my application,
// I made these functions.
// Also official LEDC driver doesn't have timer overflow interrupt.
#include <soc/ledc_reg.h>
#include <soc/ledc_struct.h>
#include "my_ledc.h"

static esp_err_t myledc_init(void);
static esp_err_t myledc_enable_intr_type(ledc_channel_t, ledc_intr_type_t);
static esp_err_t myledc_enable_intr_ovf(ledc_channel_t channel, myledc_intr_type_t enable);
static uint8_t initialized = false;

typedef struct MyLedcCallback {
	myledc_isr_function	isr_function;
	void*				arg;
} MyLedcCallback;

enum {
	MYLEDC_CALLBACK_MAX	= 4,
};

static volatile MyLedcCallback myledcCallbacks[MYLEDC_CALLBACK_MAX] = { 0 };
static volatile uint32_t enabledCallbackBitMask = 0;


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
	, myledc_intr_type_t	enable_overflow_interrupt
) {
	if(!initialized) {
		initialized = true;
		myledc_init();
	}
    if(div_param < 256) {
        return MY_ESP_ERR_LEDC_FREQUENCY_IS_TOO_HIGH;
    }
    if(div_param > 0x3ffff) {
        return MY_ESP_ERR_LEDC_FREQUENCY_IS_TOO_LOW;
    }

    ledc_timer_set(speed_mode, timer_num, div_param, duty_resolution, timer_clk_src);
    ledc_timer_rst(speed_mode, timer_num);

    ledc_set_duty(speed_mode, ledc_channel, duty);
    ledc_update_duty(speed_mode, ledc_channel);
    ledc_bind_channel_timer(speed_mode, ledc_channel, timer_num);
    myledc_enable_intr_type(ledc_channel, intr_type);
	myledc_enable_intr_ovf(ledc_channel, enable_overflow_interrupt);

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
	, myledc_intr_type_t	enable_overflow_interrupt
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
		, enable_overflow_interrupt
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


static esp_err_t myledc_enable_intr_ovf(
      ledc_channel_t channel
    , myledc_intr_type_t type
) {
    const uint32_t value = LEDC.int_ena.val;
    const uint32_t mask = BIT(LEDC_HSTIMER0_OVF_INT_ENA_S + channel);
	if (type == MYLEDC_INTR_OVERFLOW) {
        LEDC.int_ena.val = value | mask;
    } else {
        LEDC.int_ena.val = value & (~mask);
    }

	return ESP_OK;
}


esp_err_t myledc_set_ovf_isr_handler(
	  ledc_timer_t			timer_num
	, myledc_isr_function	isr_function
	, void*					user_arg
) {
	volatile MyLedcCallback* lc = &myledcCallbacks[timer_num];
	const uint32_t mask = BIT(LEDC_HSTIMER0_OVF_INT_ST_S) << timer_num;
	if(isr_function == NULL) {
		lc->isr_function = NULL;
		enabledCallbackBitMask &= ~mask;
	} else {
		lc->isr_function	= isr_function;
		lc->arg				= user_arg;
		enabledCallbackBitMask |= mask;
	}
	return ESP_OK;
}


static void IRAM_ATTR myledc_isr(void* arg) {
	const uint32_t intr_status = LEDC.int_st.val;

	uint32_t status = intr_status & enabledCallbackBitMask;
	if(status == 0) {
		// TODO : The following code clears all interrupt status.
		// But it should be clear bits which are related to ISR.
		// Find good way to cooperate with official LEDC driver.
		LEDC.int_clr.val = intr_status;				//clear LEDC interrupt status.
		return;
	}

	int channel = 0;
	uint32_t s = status;
	for(; s != 0; s >>= 1, ++channel) {
		const uint32_t mask = BIT(LEDC_HSTIMER0_OVF_INT_ST_S);
		if(s & mask) {
			volatile MyLedcCallback* p = &myledcCallbacks[channel];
			p->isr_function(p->arg);
		}
	}

	LEDC.int_clr.val = status;
}


static esp_err_t myledc_init(void) {
    periph_module_enable(PERIPH_LEDC_MODULE);
	ledc_isr_handle_t isrHandle;
	ledc_isr_register(myledc_isr, NULL, ESP_INTR_FLAG_IRAM, &isrHandle);
	return ESP_OK;
}
