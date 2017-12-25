#ifndef MY_CONFIG_H
#define MY_CONFIG_H

#include <esp_err.h>

// GPIO Pins
#define YOUR_GPIO_NUM_HSYNC		GPIO_NUM_23			// OUT: VGA H-SYNC
#define YOUR_GPIO_NUM_VSYNC		GPIO_NUM_22			// OUT: VGA V-SYNC
#define YOUR_GPIO_NUM_VIDEO		GPIO_NUM_25			// OUT: VGA Video

// Peripherals
#define YOUR_LEDC_TIMER_HSYNC	LEDC_TIMER_0		// High-speed timer (LEDC_TIMER_x, x:[0,3])
#define YOUR_LEDC_CHANNEL_HSYNC	LEDC_CHANNEL_0		// High-speed timer (LEDC_CHANNEL_x, x:[0,3])
#define YOUR_LEDC_TIMER_VSYNC	LEDC_TIMER_1		// High-speed timer (LEDC_TIMER_x, x:[0,3])
#define YOUR_LEDC_CHANNEL_VSYNC	LEDC_CHANNEL_1		// High-speed timer (LEDC_CHANNEL_x, x:[0,3])
#define YOUR_SPI_HOST			HSPI_HOST			// SPI Host (HSPI or VSPI)
#define YOUR_SPI_DMA_CHANNEL	1


#define MY_ESP_ERR(base,x)      ((esp_err_t) ((base) + (x)))

#define MY_ESP_ERR_USER_BASE    (0x40000000)
#define MY_ESP_ERR_LEDC_BASE    (MY_ESP_ERR_USER_BASE + 0x1000 * 0)
#define MY_ESP_ERR_SPI_BASE     (MY_ESP_ERR_USER_BASE + 0x1000 * 1)
#define MY_ESP_ERR_VGA_BASE     (MY_ESP_ERR_USER_BASE + 0x1000 * 2)

#endif
