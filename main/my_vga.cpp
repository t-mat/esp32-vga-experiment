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
#include <soc/ledc_reg.h>
#include "my_ledc.h"
#include "my_spi.h"
#include "my_vga.h"


//
//  GPIO Pin definitions
//		Configure these values by editing my_config.h
//
namespace GpioPins {
    const gpio_num_t		Hsync			= static_cast<gpio_num_t>(YOUR_GPIO_NUM_HSYNC);  // OUT: VGA H-SYNC
    const gpio_num_t		Vsync			= static_cast<gpio_num_t>(YOUR_GPIO_NUM_VSYNC);  // OUT: VGA V-SYNC
    const gpio_num_t		SpiMosi			= static_cast<gpio_num_t>(YOUR_GPIO_NUM_VIDEO);  // OUT: VGA Video
}


//
//  Peripheral definitions
//		Configure these values by editing my_config.h
//
namespace Peripherals {
	const ledc_timer_t      HTimer			= static_cast<ledc_timer_t>(YOUR_LEDC_TIMER_HSYNC);
	const ledc_channel_t    HChannel		= static_cast<ledc_channel_t>(YOUR_LEDC_CHANNEL_HSYNC);
	const ledc_timer_t		VTimer			= static_cast<ledc_timer_t>(YOUR_LEDC_TIMER_VSYNC);
	const ledc_channel_t    VChannel		= static_cast<ledc_channel_t>(YOUR_LEDC_CHANNEL_VSYNC);
	const spi_host_device_t SpiHost			= static_cast<spi_host_device_t>(YOUR_SPI_HOST);
	const int               SpiDmaChannel	= static_cast<int>(YOUR_SPI_DMA_CHANNEL);
}


namespace {
//
//  800x600@60Hz, PixelFrequency = 40 MHz
//
const double	VgaPixelFrequencyInHz	= 40.0 * 1000.0 * 1000.0;	// 40.0 MHz
const bool      VSyncPolarity       	= true;						//  true:sync=HI, false:sync=LO
const bool      HSyncPolarity       	= true;						//  true:sync=HI, false:sync=LO
const double    SpiDmaClockSpeedInHz    = VgaPixelFrequencyInHz;	// 40.0 MHz

const int	VgaVideoWidth				= 800;
const int	VgaVideoHeight				= 600;

const int	VgaHSyncFrontPorchInPixels	= 40;
const int	VgaHSyncSignalInPixels		= 128;
const int	VgaHSyncBackPorchInPixels	= 88;

const int	VgaVSyncFrontPorchInLines	= 1;
const int	VgaVSyncSignalInLines		= 4;
const int	VgaVSyncBackPorchInLines	= 23;

const int	VgaSignalWidthInPixels =	// 1056 pixels / line
				  VgaHSyncFrontPorchInPixels
				+ VgaHSyncSignalInPixels
				+ VgaHSyncBackPorchInPixels
				+ VgaVideoWidth;

const int	VgaSignalHeightInLines =	// 628 lines / frame
				  VgaVSyncFrontPorchInLines
				+ VgaVSyncSignalInLines
				+ VgaVSyncBackPorchInLines
				+ VgaVideoHeight;

const int	VgaTotalScreenAreaInPixels			= VgaSignalWidthInPixels * VgaSignalHeightInLines;
const int	VgaTotalScreenAreaForVsyncInPixels  = VgaSignalWidthInPixels * VgaVSyncSignalInLines;
const double	VgaVSyncDuty			= (double) VgaTotalScreenAreaForVsyncInPixels / (double) VgaTotalScreenAreaInPixels;
const double	VgaHSyncDuty			= (double) VgaHSyncSignalInPixels / (double) VgaSignalWidthInPixels;
const double	VgaVSyncFrequencyInHz	= (double) VgaPixelFrequencyInHz / (double) VgaTotalScreenAreaInPixels;
const double	VgaHSyncFrequencyInHz	= (double) VgaPixelFrequencyInHz / (double) VgaSignalWidthInPixels;

const int	SpiHSyncBackporchWaitCycle	= 44;		// H-SYNC backporch : 2.16us
} // anonymous namespace


class VgaContext {
public:
	using ThisClass = VgaContext;

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
		extraSize += sizeof(lldesc_t) * 2 * VgaSignalHeightInLines;
		return extraSize;
	}

	esp_err_t cleanup() {
		return ESP_OK;
	}

	esp_err_t init(const myvga_init_params_t* initParams) {
		auto* ap = reinterpret_cast<uint8_t*>(this);
		ap += sizeof(ThisClass);

		frameCount = 0;

		{
			const auto& ip = *initParams;
			userVideo.width		= ip.video.width;
			userVideo.height	= ip.video.height;
			userVideo.stride	= ip.video.strideInBytes;
			userVideo.buffer	= static_cast<uint8_t*>(ip.video.buffer);
		}

		vsyncCallback.callback	= nullptr;

		blankLine	= reinterpret_cast<decltype(blankLine)>(ap);
		ap += blankLineBytes;
		{
			memset(blankLine, 0, blankLineBytes);
		}

		descs		= reinterpret_cast<decltype(descs)>(ap);
		ap += sizeof(lldesc_t) * 2 * VgaSignalHeightInLines;
		{
			for(int y = 0; y < VgaSignalHeightInLines; ++y) {
				const int videoY = y - (VgaVSyncSignalInLines + VgaVSyncBackPorchInLines);
				const bool isVideoEnable = (videoY >= 0 && videoY < userVideo.height);
				const bool isLast = (y == VgaSignalHeightInLines - 1);
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
					const int dmaChunkLen = (VgaSignalWidthInPixels - userVideo.width) / 8;
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

		setupAndKickTimerAndDma();

		frameCount = 0;
		return ESP_OK;
	}

	void setupAndKickTimerAndDma() {
		myspi_prepare_circular_buffer(
		      Peripherals::SpiHost
		    , Peripherals::SpiDmaChannel
		    , descs
		    , SpiDmaClockSpeedInHz
		    , GpioPins::SpiMosi
		    , SpiHSyncBackporchWaitCycle
		);

		// VSYNC Pulse
		{
		    const double            freq_hz         = VgaVSyncFrequencyInHz;
		    const ledc_timer_bit_t  duty_resolution = LEDC_TIMER_11_BIT;
		    const ledc_timer_t      timer_num       = Peripherals::VTimer;

		    const gpio_num_t        gpio_num        = GpioPins::Vsync;
			const double            dutyRatio       = VSyncPolarity ? VgaVSyncDuty : 1.0 - VgaVSyncDuty;
		    const ledc_channel_t    ledc_channel    = Peripherals::VChannel;
			const myledc_intr_type_t	enable_overflow_interrupt	= MYLEDC_INTR_OVERFLOW;

		    myledc_set_frequency_and_duty(
		          gpio_num
		        , freq_hz
		        , dutyRatio
		        , duty_resolution
		        , timer_num
		        , ledc_channel
				, enable_overflow_interrupt
		    );

			myledc_set_ovf_isr_handler(
				  timer_num
				, vsyncIsr
				, reinterpret_cast<void*>(this)
			);
		}

		// HSYNC Pulse
		{
		    const double            freq_hz         = VgaHSyncFrequencyInHz;
		    const ledc_timer_bit_t  duty_resolution = LEDC_TIMER_10_BIT;
			const ledc_timer_t      timer_num       = Peripherals::HTimer;

		    const gpio_num_t        gpio_num        = GpioPins::Hsync;
			const double            dutyRatio       = HSyncPolarity ? VgaHSyncDuty : 1.0 - VgaHSyncDuty;
		    const ledc_channel_t    ledc_channel    = Peripherals::HChannel;
			const myledc_intr_type_t	enable_overflow_interrupt	= MYLEDC_INTR_OVERFLOW;

		    myledc_set_frequency_and_duty(
		          gpio_num
		        , freq_hz
		        , dutyRatio
		        , duty_resolution
		        , timer_num
		        , ledc_channel
				, enable_overflow_interrupt
		    );

			myledc_set_ovf_isr_handler(
				  timer_num
				, hsyncIsr
				, reinterpret_cast<void*>(this)
			);
		}

		portDISABLE_INTERRUPTS();
		kickTimerAndDma();
		portENABLE_INTERRUPTS();
	}

	void IRAM_ATTR kickTimerAndDma() {
		// Reset timers and begin SPI DMA transfer
		const ledc_mode_t       ledc_speed_mode     = LEDC_HIGH_SPEED_MODE;
		spi_dev_t* const spiHw = myspi_get_hw_for_host(Peripherals::SpiHost);

		const uint32_t v_timer_conf = LEDC.timer_group[ledc_speed_mode].timer[Peripherals::VTimer].conf.val;
		const uint32_t h_timer_conf = LEDC.timer_group[ledc_speed_mode].timer[Peripherals::HTimer].conf.val;

		const uint32_t rst_mask = LEDC_HSTIMER0_RST;

		const uint32_t v_timer_conf_rst0 = v_timer_conf & ~rst_mask;
		const uint32_t h_timer_conf_rst0 = h_timer_conf & ~rst_mask;
		const uint32_t v_timer_conf_rst1 = v_timer_conf |  rst_mask;
		const uint32_t h_timer_conf_rst1 = h_timer_conf |  rst_mask;

		LEDC.timer_group[ledc_speed_mode].timer[Peripherals::VTimer].conf.val = v_timer_conf_rst1;
		LEDC.timer_group[ledc_speed_mode].timer[Peripherals::HTimer].conf.val = h_timer_conf_rst1;

//		uint32_t c0;
//		asm volatile("rsr %0, ccount" : "=a"(c0));

		// Start H-Sync timer
		LEDC.timer_group[ledc_speed_mode].timer[Peripherals::HTimer].conf.val = h_timer_conf_rst0;

		// Start SPI DMA transfer (2)
		spiHw->cmd.usr = 1;
		// Start SPI DMA transfer (1)
		spiHw->dma_out_link.start   = 1;

		// Make H-Sync backporch
		//      TODO : Find more stable way for waiting.
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");

//		https://github.com/espressif/arduino-esp32/issues/282#issuecomment-289093098
//		const uint32_t	vsyncWaitCycle	= 0;
//		const uint32_t	timeout			= c0 + vsyncWaitCycle;
//		for(;;) {
//			uint32_t c1;
//			asm volatile("rsr %0, ccount" : "=a"(c1));
//			if(static_cast<int32_t>(timeout - c1) <= 0) {
//				break;
//			}
//		}

		// Start V-Sync timer
		LEDC.timer_group[ledc_speed_mode].timer[Peripherals::VTimer].conf.val = v_timer_conf_rst0;
	}

	int32_t getFrameCount32() const {
		return static_cast<int32_t>(getFrameCount64());
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
		if(vsyncCallback.callback) {
			vsyncCallback.callback(vsyncCallback.userPtr);
		}
	}

	static void vsyncIsr(void* p) {
		reinterpret_cast<ThisClass*>(p)->onVsync();
		return;
	}

	void onHsync() {
	}

	static void hsyncIsr(void* p) {
		reinterpret_cast<ThisClass*>(p)->onHsync();
		return;
	}

	esp_err_t setVsyncCallback(myvga_vsync_callback callback, void* userPtr) {
		vsyncCallback.callback = callback;
		vsyncCallback.userPtr = userPtr;
		return ESP_OK;
	}

	struct {
		int16_t		width;		// width in pixels
		int16_t		height;		// heigt in pixels
		int16_t		stride;		// stride in bytes
		uint8_t*	buffer;		// buffer pointer must be aligned to 4 bytes.
	} userVideo;

	struct {
		myvga_vsync_callback	callback;
		void*					userPtr;
	} vsyncCallback;

	static const size_t blankLineBytes	= VgaSignalWidthInPixels / 8;
	static VgaContext* ctx;

	int64_t				frameCount;
	uint8_t*			blankLine;
	lldesc_t*			descs;
};

VgaContext* VgaContext::ctx;

#define CTX	VgaContext::ctx


void myvga_prepare_init_init_params(
	myvga_init_params_t* initParams
) {
	return VgaContext::initInitParams(initParams);
}

size_t myvga_prepare_get_memory_size(
	const myvga_init_params_t* initParams
) {
	return VgaContext::calcMemorySize(initParams);
}


esp_err_t myvga_init(
	  const myvga_init_params_t* initParams
	, void* dedicatedMemoryForMyvga
	, size_t dedicatedMemoryForMyvgaInBytes
) {
	CTX = (VgaContext*) dedicatedMemoryForMyvga;
	return CTX->init(initParams);
}


esp_err_t myvga_cleanup(void) {
	const auto result = CTX->cleanup();
	CTX = nullptr;
	return result;
}


int32_t myvga_get_frame_count32(void) {
	return CTX->getFrameCount32();
}


int64_t myvga_get_frame_count64(void) {
	return CTX->getFrameCount64();
}


esp_err_t myvga_set_vsync_callback_function(myvga_vsync_callback callback, void* user_ptr) {
	return CTX->setVsyncCallback(callback, user_ptr);
}
