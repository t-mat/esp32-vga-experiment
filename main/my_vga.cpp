#include <string.h>
#include <stdint.h>

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <soc/spi_reg.h>
#include <soc/dport_reg.h>

#include "my_spi.h"
#include "my_rmt.h"
#include "my_vga.h"


namespace {
//
//  800x600@60Hz, PixelFrequency = 40 MHz
//
//		http://tinyvga.com/vga-timing/800x600@60Hz
//
//	Pixel frequency:  40.0 MHz
//
//	Horizontal timing (line)
//
//		Polarity of horizontal sync pulse is positive.
//		(H-Sync pulse is +V, non-pulse is 0V)
//
//		Visible area	 800 pixels		20000.0 ns
//		Front porch		  40 pixels		 1000.0 ns
//		Sync pulse		 128 pixels		 3200.0 ns
//		Back porch		  88 pixels		 2200.0 ns
//		Whole line		1056 pixels		26400.0 ns
//
//		note: All above nano-seconds are exact nano-seconds.
//		      Thus "Whole line" requires 26400.000...0 nano-seconds.
//
//	Vertical timing (frame)
//
//		Polarity of vertical sync pulse is positive.
//		(V-Sync pulse is +V, non-pulse is 0V)
//
//		Visible area	600 lines		15840.00 us
//		Front porch		  1 line		   26.40 us
//		Sync pulse		  4 lines		  105.60 us
//		Back porch		 23 lines		  607.20 us
//		Whole frame		628 lines		16579.20 us
//
//		note: All above micro-seconds are exact micro-seconds.
//		      Thus "Whole frame" requires 16579.200...0 micro-seconds.
//

const double	VgaPixelFrequencyInHz	= 40.0 * 1000.0 * 1000.0;	// 40.0 MHz
const bool      VSyncPolarity       	= true;						//  true:sync=HI, false:sync=LO
const bool      HSyncPolarity       	= true;						//  true:sync=HI, false:sync=LO

const int		VgaVideoWidth				= 800;
const int		VgaVideoHeight				= 600;

const int		VgaHSyncFrontPorchInPixels	= 40;
const int		VgaHSyncSignalInPixels		= 128;
const int		VgaHSyncBackPorchInPixels	= 88;

const int		VgaVSyncFrontPorchInLines	= 1;
const int		VgaVSyncSignalInLines		= 4;
const int		VgaVSyncBackPorchInLines	= 23;

const int		VgaSignalWidthInPixels =	// 1056 pixels / line
					  VgaHSyncFrontPorchInPixels
					+ VgaHSyncSignalInPixels
					+ VgaHSyncBackPorchInPixels
					+ VgaVideoWidth;

const int		VgaSignalHeightInLines =	// 628 lines / frame
					  VgaVSyncFrontPorchInLines
					+ VgaVSyncSignalInLines
					+ VgaVSyncBackPorchInLines
					+ VgaVideoHeight;

const int		VgaTotalScreenAreaInPixels			= VgaSignalWidthInPixels * VgaSignalHeightInLines;
const int		VgaTotalScreenAreaForVsyncInPixels  = VgaSignalWidthInPixels * VgaVSyncSignalInLines;
const double	VgaVSyncDuty			= (double) VgaTotalScreenAreaForVsyncInPixels / (double) VgaTotalScreenAreaInPixels;
const double	VgaHSyncDuty			= (double) VgaHSyncSignalInPixels / (double) VgaSignalWidthInPixels;
const double	VgaVSyncFrequencyInHz	= (double) VgaPixelFrequencyInHz / (double) VgaTotalScreenAreaInPixels;
const double	VgaHSyncFrequencyInHz	= (double) VgaPixelFrequencyInHz / (double) VgaSignalWidthInPixels;

const int		SpiHSyncBackporchWaitCycle	= 58;		// H-Sync back porch : 2.20us

const int		RmtItem32Max	= 64;
const int		RmtItem16Max	= RmtItem32Max * 2;
const int		RmtDurationMax	= 32767;

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
		size_t extraSize = sizeof(ThisClass);						// DRAM
		extraSize += blankLineBytes;								// DRAM, DMA
		extraSize += sizeof(lldesc_t) * 2 * VgaSignalHeightInLines;	// DRAM, DMA
		return extraSize;
	}

	esp_err_t cleanup() {
		// TODO : Stop SPI DMA
		// TODO : Stop RMT Channels
		// TODO : Remove RMT ISRs
		// TODO : Disable RMT interrupts
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

			spi.host			= ip.spi.host;
			spi.dmaChan			= ip.spi.dmaChan;
			spi.mosiGpioNum		= ip.spi.mosiGpioNum;
			spi.hw				= myspi_get_hw_for_host(ip.spi.host);

			rmt.hsyncChannel	= ip.rmt.hsyncChannel;
			rmt.vsyncChannel	= ip.rmt.vsyncChannel;
			rmt.hsyncGpioNum	= ip.rmt.hsyncGpioNum;
			rmt.vsyncGpioNum	= ip.rmt.vsyncGpioNum;
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

		// setup RMT for H-Sync
		myrmt_setup_pulse_output(
			  rmt.hsyncChannel
			, rmt.hsyncGpioNum
			, 1
			, 270
			, 1
			, 1837
			, 0
			, nullptr
		);

		// setup RMT for V-Sync
		{
			int32_t nItems = 0;
			myrmt_setup_pulse_output(
				  rmt.vsyncChannel
				, rmt.vsyncGpioNum
				, 4
				, 878
				, 1
				, 330704
				, 0
				, &nItems
			);
		}

		const double SpiDmaClockSpeedInHz = VgaPixelFrequencyInHz;

		myspi_prepare_circular_buffer(
		      spi.host
		    , spi.dmaChan
		    , descs
		    , SpiDmaClockSpeedInHz
		    , spi.mosiGpioNum
		    , SpiHSyncBackporchWaitCycle
		);

		intr_handle_t my_rmt_isr_handle;
		ESP_ERROR_CHECK(esp_intr_alloc(ETS_RMT_INTR_SOURCE, ESP_INTR_FLAG_SHARED, rmtIsr, this, &my_rmt_isr_handle));

		portDISABLE_INTERRUPTS();
		// Reset timers and begin SPI DMA transfer
		spi_dev_t* const spiHw = getSpiHw();

		// Here, we're waiting for completion of RMT TX.  When TX is completed,
		// RMT channel's internal counter becomes some constant value (maybe 0?).
		// Therefore, we can see stable behaviour of RMT channel.
		{
			auto& hsyncRmtConf1 = RMT.conf_ch[rmt.hsyncChannel].conf1;
			auto& vsyncRmtConf1 = RMT.conf_ch[rmt.vsyncChannel].conf1;

			hsyncRmtConf1.tx_conti_mode	= 0;
			vsyncRmtConf1.tx_conti_mode	= 0;

			const uint32_t mask = BIT(rmt.hsyncChannel * 3 + 0) | BIT(rmt.vsyncChannel * 3 + 0);
			for(;;) {
				const uint32_t int_raw = RMT.int_raw.val;
				if((int_raw & mask) == mask) {
					break;
				}
			}

			hsyncRmtConf1.ref_cnt_rst	= 1;	// RMT_REF_CNT_RST_CH	Setting this bit resets the clock divider of channel n. (R/W)
			vsyncRmtConf1.ref_cnt_rst	= 1;	// RMT_REF_CNT_RST_CH

			hsyncRmtConf1.mem_rd_rst	= 1;	// RMT_MEM_RD_RST_CHn	Set this bit to reset the read-RAM address for channel n by accessing the transmitter. (R/W)
			vsyncRmtConf1.mem_rd_rst	= 1;	// RMT_MEM_RD_RST_CHn
		}


		spiHw->dma_conf.dma_tx_stop		= 1;	// Stop SPI DMA
		spiHw->ctrl2.val           		= 0;	// Reset timing
		spiHw->dma_conf.dma_tx_stop		= 0;	// Disable stop
		spiHw->dma_conf.dma_continue	= 1;	// Set contiguous mode
		spiHw->dma_out_link.start		= 1;	// Start SPI DMA transfer (1)

		ESP_ERROR_CHECK(rmt_set_tx_thr_intr_en(rmt.hsyncChannel, true, 1));
		ESP_ERROR_CHECK(rmt_set_tx_thr_intr_en(rmt.vsyncChannel, true, 7));

		clearIsrCounters();

		kickPeripherals(spiHw, rmt.hsyncChannel, rmt.vsyncChannel);
		portENABLE_INTERRUPTS();

		return ESP_OK;
	}

	static void IRAM_ATTR kickPeripherals(
		  spi_dev_t* spiHw
		, rmt_channel_t hsyncChannel
		, rmt_channel_t vsyncChannel
	) {
		auto& hsyncRmtConf1 = RMT.conf_ch[hsyncChannel].conf1;
		auto& vsyncRmtConf1 = RMT.conf_ch[vsyncChannel].conf1;

		hsyncRmtConf1.tx_conti_mode		= 1;	// RMT: Set this bit to start sending data on channel n, in contiguous mode.
		vsyncRmtConf1.tx_conti_mode		= 1;	// RMT: Set this bit to start sending data on channel n, in contiguous mode.
		spiHw->cmd.usr					= 1;	// SPI: Start SPI DMA transfer
	}

	lldesc_t* getLldescs() {
		return descs;
	}

	spi_dev_t* getSpiHw() {
		return spi.hw;
	}

	int32_t getFrameCount32() const {
		return static_cast<int32_t>(getFrameCount64());
	}

	int64_t getFrameCount64() const {
		return isrCounters.vcounterEvt;
	}

	void onVsync() {
		isrCounters.vcounterEvt += 1;
		if(vsyncCallback.callback) {
			vsyncCallback.callback(vsyncCallback.userPtr);
		}
	}

	static void IRAM_ATTR vsyncIsr(void* p) {
		reinterpret_cast<ThisClass*>(p)->onVsync();
	}

	void onHsync() {
		isrCounters.hcounterEvt += 1;
	}

	static void IRAM_ATTR hsyncIsr(void* p) {
		reinterpret_cast<ThisClass*>(p)->onHsync();
	}

	static void rmtIsr(void* p) {
		reinterpret_cast<ThisClass*>(p)->onRmtEvent();
	}

	esp_err_t setVsyncCallback(myvga_vsync_callback callback, void* userPtr) {
		vsyncCallback.callback = callback;
		vsyncCallback.userPtr = userPtr;
		return ESP_OK;
	}

	void clearIsrCounters() {
		isrCounters.vcounterEvt = 0;
		isrCounters.hcounterEvt = 0;
	}

	void onRmtEvent() {
		const uint32_t intr_st = RMT.int_st.val;

		if(intr_st & BIT(rmt.vsyncChannel + 24)) {
			onVsync();
		}

		if(intr_st & BIT(rmt.hsyncChannel + 24)) {
			onHsync();
		}
	}

	struct {
		int16_t		width;		// width in pixels
		int16_t		height;		// heigt in pixels
		int16_t		stride;		// stride in bytes
		uint8_t*	buffer;		// buffer pointer must be aligned to 4 bytes.
	} userVideo;

	struct {
		spi_host_device_t	host;			// HSPI_HOST or VSPI_HOST
		int					dmaChan;		// 0, 1 or 2
		gpio_num_t			mosiGpioNum;	// GPIO
		spi_dev_t*			hw;
	} spi;

	struct {
		rmt_channel_t		hsyncChannel;	// [0,7]
		rmt_channel_t		vsyncChannel;	// [0,7]
		gpio_num_t			hsyncGpioNum;	// GPIO
		gpio_num_t			vsyncGpioNum;	// GPIO
	} rmt;

	struct {
		myvga_vsync_callback	callback;
		void*					userPtr;
	} vsyncCallback;

	struct {
		volatile int64_t vcounterEvt;
		volatile int32_t hcounterEvt;
	} isrCounters;

	static const size_t blankLineBytes	= VgaSignalWidthInPixels / 8;
	static VgaContext* ctx;

	uint8_t*			blankLine;
	lldesc_t*			descs;
};


////////////////////////////////////////////////////////////////////////////////////////////
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
