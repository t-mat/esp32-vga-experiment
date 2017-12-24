#ifndef MY_VGA_H
#define MY_VGA_H

//#include <driver/spi_common.h>
//#include <driver/spi_master.h>
#include "my_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MY_ESP_ERR_VGA(x)                       MY_ESP_ERR(MY_ESP_ERR_VGA_BASE, (x))

typedef struct myvga_init_params_t myvga_init_params_t;

struct myvga_init_params_t {
	uint32_t		flags;
	struct {
		uint16_t	flags;
		int16_t		width;
		int16_t		height;
		uint16_t	strideInBytes;
		size_t		bufferSizeInBytes;
		void*		buffer;
	} video;
};

void myvga_prepare_init_init_params(myvga_init_params_t* initParams);
size_t myvga_prepare_get_memory_size(const myvga_init_params_t* initParams);

esp_err_t myvga_init(const myvga_init_params_t* initParams, void* dedicatedMemoryForMyvga, size_t dedicatedMemoryForMyvgaInBytes);
//esp_err_t myvga_cleanup(void);


#ifdef __cplusplus
} // extern "C"
#endif

#endif
