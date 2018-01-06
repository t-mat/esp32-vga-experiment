//	Please edit GpioPins {}, Peripherals {} and connect GPIO pins.
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <driver/spi_common.h>

#include "my_vga.h"


///////////////////////////////////////////////////////////////////////////
//
//	Set, modify and connect the following GpioPins and Peripherals.
//
///////////////////////////////////////////////////////////////////////////
namespace GpioPins {
	const gpio_num_t		outHsync		= static_cast<gpio_num_t>(GPIO_NUM_23);	// OUT: VGA H-SYNC from RMT
	const gpio_num_t		outVsync		= static_cast<gpio_num_t>(GPIO_NUM_22);	// OUT: VGA V-SYNC from RMT
	const gpio_num_t		outVideo		= static_cast<gpio_num_t>(GPIO_NUM_25);	// OUT: VGA VIDEO from SPI
	const gpio_num_t		outLedBlink		= static_cast<gpio_num_t>(GPIO_NUM_2);	// OUT: LED heartbeat
}

namespace Peripherals {
	const spi_host_device_t SpiHost			= static_cast<spi_host_device_t>(HSPI_HOST);	// HSPI or VSPI
	const int               SpiDmaChannel	= static_cast<int>(1);		// 1 or 2
	const rmt_channel_t		RmtChannelHsync	= RMT_CHANNEL_0;			// RMT Channel for H-SYNC signal
	const rmt_channel_t		RmtChannelVsync	= RMT_CHANNEL_1;			// RMT Channel for V-SYNC signal
}


///////////////////////////////////////////////////////////////////////////
enum {
	VIDEO_WIDTH		= 800,
	VIDEO_HEIGHT	= 600,
	VIDEO_STRIDE	= VIDEO_WIDTH / 8,
};

static uint8_t*	videoBuf	= NULL;
static void*	vgaBuf		= NULL;


///////////////////////////////////////////////////////////////////////////
static void heartBeat(gpio_num_t outLed) {
	static int8_t counter = -1;
	if(counter < 0) {
		counter = 0;
		gpio_pad_select_gpio(outLed);
		gpio_set_direction(outLed, GPIO_MODE_OUTPUT);
	}

	counter = (counter + 1) & 127;
	if((counter & 31) == 0) {
		gpio_set_level(outLed, (counter & 32) == 0);
	}
}


///////////////////////////////////////////////////////////////////////////
// http://www.pcg-random.org/download.html
typedef struct { uint64_t state;  uint64_t inc; } pcg32_random_t;

static uint32_t pcg32_random_r(pcg32_random_t* rng) {
    uint64_t oldstate = rng->state;
    // Advance internal state
    rng->state = oldstate * 6364136223846793005ULL + (rng->inc|1);
    // Calculate output function (XSH RR), uses old state for max ILP
    uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
    uint32_t rot = oldstate >> 59u;
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

static uint32_t rnd(int x) {
	static pcg32_random_t rng = { 0, 0 };
	return pcg32_random_r(&rng) % x;
}


///////////////////////////////////////////////////////////////////////////
static void setPixel(int x, int y, int color) {
	uint8_t* p = videoBuf + x/8 + y * VIDEO_STRIDE;
	if(color) {
		const uint8_t mask = 1 << (x & 7);
		*p |= mask;
	} else {
		const uint8_t mask = 0xfe << (x & 7);
		*p &= mask;
	}
}

static void reversePixel(int x, int y) {
	uint8_t* p = videoBuf + x/8 + y * VIDEO_STRIDE;
	const uint8_t mask = 1 << (x & 7);
	*p ^= mask;
}

static void clearAllPixels(int c = 0) {
	memset(videoBuf, c, VIDEO_STRIDE * VIDEO_HEIGHT);
}


///////////////////////////////////////////////////////////////////////////
static void onVsync() {
	static int cnt = 0;
	if(--cnt <= 0) {
		cnt = 300;	//rnd(150) + 300;
		clearAllPixels();
	}

	for(int i = 0; i < 1000; ++i) {
		reversePixel(rnd(VIDEO_WIDTH), rnd(VIDEO_HEIGHT));
	}

	for(int x = 0; x < VIDEO_WIDTH; ++x) {
		setPixel(x, 0, 1);
		setPixel(x, VIDEO_HEIGHT-1, 1);
	}

	for(int y = 0; y < VIDEO_HEIGHT; ++y) {
		setPixel(0, y, 1);
		setPixel(VIDEO_WIDTH-1, y, 1);
	}
}


///////////////////////////////////////////////////////////////////////////
static SemaphoreHandle_t vsyncSemaphore = nullptr;
static volatile int32_t onVsyncCount = 0;


static void onVsyncIsr(void* userPtr) {
	if(vsyncSemaphore == nullptr) {
		return;
	}

	++onVsyncCount;

	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(vsyncSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
}


static void taskVsync(void * pvParameters) {
    vsyncSemaphore = xSemaphoreCreateBinary();

	// Set V-Sync callback (ISR)
	myvga_set_vsync_callback_function(onVsyncIsr, NULL);

	for(;;) {
		// Wait fror V-Sync
        if(xSemaphoreTake(vsyncSemaphore, portMAX_DELAY) != pdTRUE) {
			continue;
		}
		onVsync();
    }
}


///////////////////////////////////////////////////////////////////////////
static void vga_setup() {
	myvga_init_params_t params;
	myvga_prepare_init_init_params(&params);

	const size_t videoBufSizeInBytes = VIDEO_STRIDE * VIDEO_HEIGHT;
	videoBuf = (uint8_t*) malloc(videoBufSizeInBytes);

	// Set initialize parameters and allocate memory for own video buffer.
	params.video.width				= VIDEO_WIDTH;
	params.video.height				= VIDEO_HEIGHT;
	params.video.strideInBytes		= VIDEO_STRIDE;
	params.video.bufferSizeInBytes	= videoBufSizeInBytes;
	params.video.buffer				= videoBuf;
	params.spi.host					= Peripherals::SpiHost;
	params.spi.dmaChan				= Peripherals::SpiDmaChannel;
	params.spi.mosiGpioNum			= GpioPins::outVideo;
	params.rmt.hsyncChannel			= Peripherals::RmtChannelHsync;
	params.rmt.hsyncGpioNum			= GpioPins::outHsync;
	params.rmt.vsyncChannel			= Peripherals::RmtChannelVsync;
	params.rmt.vsyncGpioNum			= GpioPins::outVsync;

	// Allocate memory for VGA class.
	const size_t vgaBufSize = myvga_prepare_get_memory_size(&params);
	vgaBuf = malloc(vgaBufSize);

	// Initialize VGA class.
	myvga_init(&params, vgaBuf, vgaBufSize);
}

///////////////////////////////////////////////////////////////////////////
static void setup() {
	vga_setup();
}

static void loop() {
	static int cc = 0;
	if(--cc < 0) {
		cc = 100;
		const int32_t	frameCount	= myvga_get_frame_count32();
		printf("frameCount=%d, onVsyncCount=%d\n", frameCount, onVsyncCount);
	}
}

extern "C" void app_main() {
	setup();
	fflush(stdout);

	TaskHandle_t xHandle = nullptr;
	//BaseType_t xReturned =
	xTaskCreate(
		  taskVsync       			// Function that implements the task.
		, "VGA-VSync"          		// Text name for the task.
		, configMINIMAL_STACK_SIZE  // Stack size in words, not bytes.
		, ( void * ) 1    			// pvParameters: Parameter passed into the task.
		, tskIDLE_PRIORITY			// Priority at which the task is created.
		, &xHandle
	);

	for(;;) {
		heartBeat(GpioPins::outLedBlink);
		vTaskDelay(10 / portTICK_RATE_MS);
		loop();
	}

	if(xHandle != NULL) {
		vTaskDelete(xHandle);
	}
}
