#include <string.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "my_vga.h"

enum {
	VIDEO_WIDTH		= 800,
	VIDEO_HEIGHT	= 600,
	VIDEO_STRIDE	= VIDEO_WIDTH / 8,
};

static uint8_t*	video_buf	= NULL;
static void*	vga_buf		= NULL;


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


//
static void setPixel(int x, int y, int color) {
	uint8_t* p = video_buf + x/8 + y * VIDEO_STRIDE;
	const uint8_t mask = 0x80 >> (x & 7);
	if(color) {
		*p |= mask;
	} else {
		*p = ~mask;
	}
}

static void reversePixel(int x, int y) {
	uint8_t* p = video_buf + x/8 + y * VIDEO_STRIDE;
	const uint8_t mask = 0x80 >> (x & 7);
	*p ^= mask;
}

static void clearAllPixels() {
	memset(video_buf, 0, VIDEO_STRIDE * VIDEO_HEIGHT);
}


//
static int32_t vCount = 0;
static void IRAM_ATTR onVsync(void*) {
	++vCount;
}


//
static void init() {
	myvga_init_params_t params;
	myvga_prepare_init_init_params(&params);

	// Set initialize parameters and allocate memory for own video buffer.
	params.video.width				= VIDEO_WIDTH;
	params.video.height				= VIDEO_HEIGHT;
	params.video.strideInBytes		= VIDEO_STRIDE;
	params.video.bufferSizeInBytes	= VIDEO_STRIDE * VIDEO_HEIGHT;
	video_buf						= (uint8_t*) malloc(params.video.bufferSizeInBytes);
	params.video.buffer				= video_buf;

	// Allocate memory for VGA class.
	size_t s = myvga_prepare_get_memory_size(&params);
	vga_buf = malloc(s);

	// Initialize VGA class.
	myvga_init(&params, vga_buf, s);

	// Set V-Sync callback (ISR)
	myvga_set_vsync_callback_function(onVsync, NULL);
}


static void update() {
	static int cnt = 0;
	if(cnt-- == 0) {
		cnt = rnd(150) + 300;
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


extern "C" void app_main() {
	init();

	fflush(stdout);
	for(;;) {
		vTaskDelay(10 / portTICK_RATE_MS);
		update();

		static uint32_t c = 0;
		if(c++ % 100 == 0) {
			printf("vCount = %d\n", vCount);
		}
	}
}
