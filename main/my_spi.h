#ifndef MY_SPI_H
#define MY_SPI_H

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include "my_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MY_ESP_ERR_SPI(x)                       MY_ESP_ERR(MY_ESP_ERR_SPI_BASE, (x))
#define MY_ESP_ERR_SPI_SPI1_IS_NOT_SUPPORTED    MY_ESP_ERR_SPI(1)
#define MY_ESP_ERR_SPI_INVALID_HOST_NUMBER      MY_ESP_ERR_SPI(2)
#define MY_ESP_ERR_SPI_INVALID_DMA_CHANNEL      MY_ESP_ERR_SPI(3)
#define MY_ESP_ERR_SPI_HOST_ALREADY_IN_USE      MY_ESP_ERR_SPI(4)
#define MY_ESP_ERR_SPI_DMA_ALREADY_IN_USE       MY_ESP_ERR_SPI(5)

esp_err_t myspi_prepare_circular_buffer(
      spi_host_device_t     host                        // HSPI_HOST or VSPI_HOST
    , int                   dma_chan                    //
    , const lldesc_t*       lldesc                      // lldesc_t which makes circular buffer
    , int                   bitsPerLoop                 // Number of bits in circular buffer
    , double                clock_speed_hz              // SPI speed in Hz
    , gpio_num_t            mosi_gpio_num               // GPIO
    , int                   wait_cycle                  // Number of wait cycle before actual transmission (1 cycle means SPI DMA's single cycle)
);

spi_dev_t *myspi_get_hw_for_host(spi_host_device_t host);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
