#ifndef MY_VGA_H
#define MY_VGA_H

#include "my_config.h"
#include <driver/rmt.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MY_ESP_ERR_VGA(x)                       MY_ESP_ERR(MY_ESP_ERR_VGA_BASE, (x))

typedef struct myvga_init_params_t myvga_init_params_t;

struct myvga_init_params_t {
	uint32_t		flags;
	struct myvga_init_params_video_t {
		uint16_t	flags;
		int16_t		width;
		int16_t		height;
		uint16_t	strideInBytes;
		size_t		bufferSizeInBytes;
		void*		buffer;
	} video;
	struct myvga_init_params_spi_t {
		spi_host_device_t	host;			// HSPI_HOST or VSPI_HOST
		int					dmaChan;		// [0,2]
		gpio_num_t			mosiGpioNum;	// GPIO
	} spi;
	struct myvga_init_params_rmt_t {
		rmt_channel_t		hsyncChannel;	// [0,7]
		rmt_channel_t		vsyncChannel;	// [0,7]
		gpio_num_t			hsyncGpioNum;	// GPIO
		gpio_num_t			vsyncGpioNum;	// GPIO
	} rmt;
};

typedef void (*myvga_vsync_callback)(void* user_ptr);

void myvga_prepare_init_init_params(myvga_init_params_t* initParams);
size_t myvga_prepare_get_memory_size(const myvga_init_params_t* initParams);

esp_err_t myvga_init(const myvga_init_params_t* initParams, void* dedicatedMemoryForMyvga, size_t dedicatedMemoryForMyvgaInBytes);
//esp_err_t myvga_cleanup(void);

esp_err_t myvga_set_vsync_callback_function(myvga_vsync_callback callback, void* user_ptr);

int32_t myvga_get_frame_count32(void);
int64_t myvga_get_frame_count64(void);


#ifdef __cplusplus
} // extern "C"
#endif

#endif
