#ifndef MY_CONFIG_H
#define MY_CONFIG_H

#include <esp_err.h>

#define YOUR_GPIO_NUM_HSYNC		GPIO_NUM_23
#define YOUR_GPIO_NUM_VSYNC		GPIO_NUM_22
#define YOUR_GPIO_NUM_VIDEO		GPIO_NUM_25



#define MY_ESP_ERR(base,x)      ((esp_err_t) ((base) + (x)))

#define MY_ESP_ERR_USER_BASE    (0x40000000)
#define MY_ESP_ERR_LEDC_BASE    (MY_ESP_ERR_USER_BASE + 0x1000 * 0)
#define MY_ESP_ERR_SPI_BASE     (MY_ESP_ERR_USER_BASE + 0x1000 * 1)
#define MY_ESP_ERR_VGA_BASE     (MY_ESP_ERR_USER_BASE + 0x1000 * 2)

#endif
