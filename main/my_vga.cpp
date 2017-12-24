/*
--------------------------------------------------------------------
800x600
http://tinyvga.com/vga-timing/800x600@60Hz

Pixel Frequency     40.0MHz

Horizontal timing
    H-Signal    Video
A   HI          Off     H-Sync              128 Pixels       3.20 usec
B   LO          Off     H-Sync back porch    88 Pixels       2.20 usec
C   LO          On      Video               800 Pixels      20.00 usec
D   LO          Off     H-Sync front porch   40 Pixels       1.00 usec
--------------------------------------------------------------------
Total                                       1056Pixels      26.40 usec

Vertical timing
    V-Signal    Video
A   HI          Off     V-Sync                4 Lines         105.6 usec
B   LO          Off     V-Sync back porch    23 Lines         607.2 usec
C   LO          On      Video               600 Lines       15840.0 usec
D   LO          Off     V-Sync front porch    1 Lines          26.4 usec
--------------------------------------------------------------------
Total                                       628 Lines       16579.2 usec
*/
#include <string.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/ledc_struct.h>
#include "my_ledc.h"
#include "my_spi.h"
#include "my_vga.h"

//
//  GPIO Pin definitions
//
namespace GpioPins {
    const gpio_num_t Hsync          = static_cast<gpio_num_t>(YOUR_GPIO_NUM_HSYNC);  // OUT: VGA H-SYNC
    const gpio_num_t Vsync          = static_cast<gpio_num_t>(YOUR_GPIO_NUM_VSYNC);  // OUT: VGA V-SYNC
    const gpio_num_t SpiMosi        = static_cast<gpio_num_t>(YOUR_GPIO_NUM_VIDEO);  // OUT: VGA Video
}

namespace {
//
//  800x600@60Hz, PixelFrequency = 40 MHz
//
const double    PixelFrequency      = 40.0 * 1000.0 * 1000.0;   // 40.0 MHz
const double    HPixelsTotal        = 1056.0;                   // 1056 pixels / line
const double    HPixelsForPulse     = 128.0;                    //  128 pixels / h-signal
const double    VLinesTotal         = 628.0;                    //  628 lines / vsync
const double    VLinesForPulse      = 4.0;                      //    4 lines / v-signal
const bool      VSyncPolarity       = true;                     //  true:sync=HI, false:sync=LO
const bool      HSyncPolarity       = true;                     //  true:sync=HI, false:sync=LO

const ledc_timer_t      VTimer              = LEDC_TIMER_1;
const ledc_channel_t    VChannel            = LEDC_CHANNEL_1;
const ledc_timer_t      HTimer              = LEDC_TIMER_0;
const ledc_channel_t    HChannel            = LEDC_CHANNEL_0;
const double            VPixelsTotal        = HPixelsTotal * VLinesTotal;
const double            VPixelsForPulse     = HPixelsTotal * VLinesForPulse;

const spi_host_device_t SpiDmaHost              = HSPI_HOST;
const int               SpiDmaChannel           = 1;
const double            SpiDmaClockSpeedInHz    = 40.0 * 1000.0 * 1000.0;   // 40.0 MHz


const double	VgaPixelFrequencyInHz		= 40.0 * 1000.0 * 1000.0;   // 40.0 MHz

const int	VgaVideoWidth				= 800;
const int	VgaVideoHeight				= 600;

const int	VgaHSyncFrontPorchInPixels	= 40;
const int	VgaHSyncSignalInPixels		= 128;
const int	VgaHSyncBackPorchInPixels	= 88;

const int	VgaVSyncFrontPorchInLines	= 1;
const int	VgaVSyncSignalInLines		= 4;
const int	VgaVSyncBackPorchInLines	= 23;

const int	VgaSignalWidth =	// 1056 pixels / line
				  VgaHSyncFrontPorchInPixels
				+ VgaHSyncSignalInPixels
				+ VgaHSyncBackPorchInPixels
				+ VgaVideoWidth;

const int	VgaSignalHeight =	// 628 lines / frame
				  VgaVSyncFrontPorchInLines
				+ VgaVSyncSignalInLines
				+ VgaVSyncBackPorchInLines
				+ VgaVideoHeight;
} // anonymous namespace


class Ctx {
	using ThisClass = Ctx;

public:
	static void initInitParams(myvga_init_params_t* initParams) {
		memset(initParams, 0, sizeof(*initParams));
		auto& ip = *initParams;
		ip.video.flags				= 0;
		ip.video.width				= 800;
		ip.video.height				= 600;
		ip.video.strideInBytes		= ip.video.width / 8;
		ip.video.bufferSizeInBytes	= ip.video.strideInBytes * ip.video.height;
		ip.video.buffer				= nullptr;
	}

	static size_t calcMemorySize(const myvga_init_params_t* initParams) {
		size_t extraSize = sizeof(ThisClass);
		extraSize += blankLineBytes;
		extraSize += sizeof(lldesc_t) * 2 * VgaSignalHeight;
		return extraSize;
	}

	esp_err_t cleanup() {
		return ESP_OK;
	}

	esp_err_t init(const myvga_init_params_t* initParams) {
		auto* ap = reinterpret_cast<uint8_t*>(this);
		ap += sizeof(ThisClass);

		{
			const auto& ip = *initParams;
			userVideo.width		= ip.video.width;
			userVideo.height	= ip.video.height;
			userVideo.stride	= ip.video.strideInBytes;
			userVideo.buffer	= static_cast<uint8_t*>(ip.video.buffer);
		}

		blankLine	= reinterpret_cast<decltype(blankLine)>(ap);
		ap += blankLineBytes;
		{
			memset(blankLine, 0, blankLineBytes);
		}

		descs		= reinterpret_cast<decltype(descs)>(ap);
		ap += sizeof(lldesc_t) * 2 * VgaSignalHeight;
		{
			for(int y = 0; y < VgaSignalHeight; ++y) {
				const int videoY = y - (VgaVSyncSignalInLines + VgaVSyncBackPorchInLines);
				const bool isVideoEnable = (videoY >= 0 && videoY < userVideo.height);
				const bool isLast = (y == VgaSignalHeight - 1);
				{
		            auto* dd = &descs[y * 2 + 0];
					auto* next = dd + 1;
					const int dmaChunkLen = userVideo.width / 8;
		            dd->size            = dmaChunkLen;
		            dd->length          = dmaChunkLen;
					uint8_t* buf = nullptr;
					if(isVideoEnable) {
						buf = &userVideo.buffer[userVideo.stride * videoY];
					}
					if(nullptr == buf) {
						buf = blankLine;
					}
					dd->buf				= buf;
		            dd->eof             = 0;
		            dd->sosf            = 0;
		            dd->owner           = 1;
		            dd->qe.stqe_next    = next;
				}
				{
		            auto* dd = &descs[y * 2 + 1];
					auto* next = dd + 1;
					if(isLast) {
						next = &descs[0];
					}
					const int dmaChunkLen = (VgaSignalWidth - userVideo.width) / 8;
		            dd->size            = dmaChunkLen;
		            dd->length          = dmaChunkLen;
					dd->buf				= blankLine;
		            dd->eof             = 0;
		            dd->sosf            = 0;
		            dd->owner           = 1;
		            dd->qe.stqe_next    = next;
				}
			}
		}

		kickTimerAndDma();

		frameCount = 0;
		return ESP_OK;
	}

	void IRAM_ATTR kickTimerAndDma() {
		const int waitCycle             = 44;		// H-SYNC backporch : 2.17us

		myspi_prepare_circular_buffer(
		      SpiDmaHost
		    , SpiDmaChannel
		    , descs
		    , SpiDmaClockSpeedInHz
		    , GpioPins::SpiMosi
		    , waitCycle
		);

		// VSYNC Pulse
		{
		    const double            freq_hz         = PixelFrequency / VPixelsTotal;
		    const ledc_timer_bit_t  duty_resolution = LEDC_TIMER_11_BIT;
		    const ledc_timer_t      timer_num       = VTimer;

		    const gpio_num_t        gpio_num        = GpioPins::Vsync;
		    const double            dutyRatio       = VSyncPolarity ? VPixelsForPulse / VPixelsTotal : (1.0 - VPixelsForPulse / VPixelsTotal);
		    const ledc_channel_t    ledc_channel    = VChannel;

		    myledc_set_frequency_and_duty(
		          gpio_num
		        , freq_hz
		        , dutyRatio
		        , duty_resolution
		        , timer_num
		        , ledc_channel
		    );
		}

		// HSYNC Pulse
		{
		    const double            freq_hz         = PixelFrequency / HPixelsTotal;
		    const ledc_timer_bit_t  duty_resolution = LEDC_TIMER_10_BIT;
		    const ledc_timer_t      timer_num       = HTimer;

		    const gpio_num_t        gpio_num        = GpioPins::Hsync;
		    const double            dutyRatio       = HSyncPolarity ? HPixelsForPulse / HPixelsTotal : (1.0 - HPixelsTotal / HPixelsForPulse);
		    const ledc_channel_t    ledc_channel    = HChannel;

		    myledc_set_frequency_and_duty(
		          gpio_num
		        , freq_hz
		        , dutyRatio
		        , duty_resolution
		        , timer_num
		        , ledc_channel
		    );
		}

		// Reset timers and begin SPI DMA transfer
		portDISABLE_INTERRUPTS();
		const ledc_mode_t       ledc_speed_mode     = LEDC_HIGH_SPEED_MODE;
		{
		    spi_dev_t* const spiHw = myspi_get_hw_for_host(HSPI_HOST);

		    LEDC.timer_group[ledc_speed_mode].timer[HTimer].conf.rst = 1;
		    LEDC.timer_group[ledc_speed_mode].timer[VTimer].conf.rst = 1;

		    // Start H-Sync timer
		    LEDC.timer_group[ledc_speed_mode].timer[HTimer].conf.rst = 0;

			// Start SPI DMA transfer (2)
		    spiHw->cmd.usr = 1;
		    // Start SPI DMA transfer (1)
		    spiHw->dma_out_link.start   = 1;

		    // Make H-Sync backporch
		    //      TODO : Find more precice/stable way for waiting.
		    {
				static volatile int c = 0;
				++c;		// 1.36us
//				++c;		// 5.36us
//				++c;		// 5.36us
//				++c;		// 5.36us
//				xtbsp_delay_ns(2200);
		    }

		    // Start V-Sync timer
		    LEDC.timer_group[ledc_speed_mode].timer[VTimer].conf.rst = 0;
		}
		portENABLE_INTERRUPTS();
	}

	int32_t getFrameCount32() const {
		return static_cast<int32_t>(frameCount);
	}

	int64_t getFrameCount64() const {
		return frameCount;
	}

	void beginFrame() {
	}

	void endFrame() {
	}

	void onVsync() {
		++frameCount;
	}

	struct {
		int16_t		width;		// width in pixels
		int16_t		height;		// heigt in pixels
		int16_t		stride;		// stride in bytes
		uint8_t*	buffer;		// buffer pointer must be aligned to 4 bytes.
	} userVideo;

	static const size_t blankLineBytes	= VgaSignalWidth / 8;

	int64_t		frameCount;
	uint8_t*	blankLine;
	lldesc_t*	descs;
};



static Ctx* ctx;

void myvga_prepare_init_init_params(
	myvga_init_params_t* initParams
) {
	return Ctx::initInitParams(initParams);
}

size_t myvga_prepare_get_memory_size(
	const myvga_init_params_t* initParams
) {
	return Ctx::calcMemorySize(initParams);
}


esp_err_t myvga_init(
	  const myvga_init_params_t* initParams
	, void* dedicatedMemoryForMyvga
	, size_t dedicatedMemoryForMyvgaInBytes
) {
	ctx = (Ctx*) dedicatedMemoryForMyvga;
	return ctx->init(initParams);
}


esp_err_t myvga_cleanup(void) {
	const auto result = ctx->cleanup();
	ctx = nullptr;
	return result;
}

int32_t myvga_get_frame_count32(void) {
	return ctx->getFrameCount32();
}

int64_t myvga_get_frame_count64(void) {
	return ctx->getFrameCount64();
}
