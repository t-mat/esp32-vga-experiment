#include <soc/spi_reg.h>
#include <soc/dport_reg.h>
#include "my_spi.h"

static const uint8_t InvalidIndex = (uint8_t) -1;

spi_dev_t *myspi_get_hw_for_host(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return &SPI1; break;
    case HSPI_HOST: return &SPI2; break;
    case VSPI_HOST: return &SPI3; break;
    default:        return NULL;  break;
    }
}


static uint8_t getSpidOutByHost(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_OUT_IDX;	break;
    case HSPI_HOST: return HSPID_OUT_IDX;	break;
    case VSPI_HOST: return VSPID_OUT_IDX;	break;
    default:        return InvalidIndex;	break;
    }
}


static uint8_t getSpidInByHost(
    spi_host_device_t host
) {
    switch(host) {
    case SPI_HOST:  return SPID_IN_IDX;		break;
    case HSPI_HOST: return HSPID_IN_IDX;	break;
    case VSPI_HOST: return VSPID_IN_IDX;	break;
    default:        return InvalidIndex;	break;
    }
}


esp_err_t myspi_prepare_circular_buffer(
      const spi_host_device_t   spiHostDevice
    , const int                 dma_chan
    , const lldesc_t*           lldescs
    , const double              dmaClockSpeedInHz
    , const gpio_num_t          mosi_gpio_num
    , const int                 waitCycle
) {
    const bool spi_periph_claimed = spicommon_periph_claim(spiHostDevice);
    if(! spi_periph_claimed) {
        return MY_ESP_ERR_SPI_HOST_ALREADY_IN_USE;
    }

    const bool dma_chan_claimed = spicommon_dma_chan_claim(dma_chan);
    if(! dma_chan_claimed) {
        spicommon_periph_free(spiHostDevice);
        return MY_ESP_ERR_SPI_DMA_ALREADY_IN_USE;
    }

    spi_dev_t* const spiHw = myspi_get_hw_for_host(spiHostDevice);
    const int Cs = 0;
	const int CsMask = 1 << Cs;

    //Use GPIO
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[mosi_gpio_num], PIN_FUNC_GPIO);
    gpio_set_direction(mosi_gpio_num, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(mosi_gpio_num, getSpidOutByHost(spiHostDevice), false, false);
    gpio_matrix_in(mosi_gpio_num, getSpidInByHost(spiHostDevice), false);

    //Select DMA channel.
    DPORT_SET_PERI_REG_BITS(
          DPORT_SPI_DMA_CHAN_SEL_REG
        , 3
        , dma_chan
        , (spiHostDevice * 2)
    );

    //Reset DMA
    spiHw->dma_conf.val        		|= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    spiHw->dma_out_link.start  		= 0;
    spiHw->dma_in_link.start   		= 0;
    spiHw->dma_conf.val        		&= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    //Reset timing
    spiHw->ctrl2.val           		= 0;

    //Disable unneeded ints
    spiHw->slave.rd_buf_done   		= 0;
    spiHw->slave.wr_buf_done   		= 0;
    spiHw->slave.rd_sta_done   		= 0;
    spiHw->slave.wr_sta_done   		= 0;
    spiHw->slave.rd_buf_inten  		= 0;
    spiHw->slave.wr_buf_inten  		= 0;
    spiHw->slave.rd_sta_inten  		= 0;
    spiHw->slave.wr_sta_inten  		= 0;
    spiHw->slave.trans_inten   		= 0;
    spiHw->slave.trans_done    		= 0;

    //Set CS pin, CS options
	spiHw->pin.master_ck_sel			&= ~CsMask;
	spiHw->pin.master_cs_pol			&= ~CsMask;

	// Set SPI Clock
	//  Register 7.7: SPI_CLOCK_REG (0x18)
	//
	//		SPI_CLK_EQU_SYSCLK
	//			In master mode, when this bit is set to 1, spi_clk is equal
	//			to system clock; when set to 0, spi_clk is divided from system
	//			clock.
	//
	//		SPI_CLKDIV_PRE
	//			In master mode, the value of this register field is the
	//			pre-divider value for spi_clk, minus one.
	//
	//		SPI_CLKCNT_N
	//			In master mode, this is the divider for spi_clk minus one.
	//			The spi_clk frequency is
	//				system_clock/(SPI_CLKDIV_PRE+1)/(SPI_CLKCNT_N+1).
	//
	//		SPI_CLKCNT_H
	//			For a 50% duty cycle, set this to floor((SPI_CLKCNT_N+1)/2-1)
	//
	//		SPI_CLKCNT_L
	//			In master mode, this must be equal to SPI_CLKCNT_N.
	{
		const double	preDivider			= 1.0;
	    const double	apbClockSpeedInHz	= APB_CLK_FREQ;
		const double	apbClockPerDmaCycle	= (apbClockSpeedInHz / preDivider / dmaClockSpeedInHz);

		const int32_t	clkdiv_pre	= ((int32_t) preDivider) - 1;
		const int32_t	clkcnt_n	= ((int32_t) apbClockPerDmaCycle) - 1;
		const int32_t	clkcnt_h	= (clkcnt_n + 1) / 2 - 1;
		const int32_t	clkcnt_l	= clkcnt_n;

		spiHw->clock.clk_equ_sysclk	= 0;
	    spiHw->clock.clkcnt_n		= clkcnt_n;
	    spiHw->clock.clkdiv_pre		= clkdiv_pre;
		spiHw->clock.clkcnt_h		= clkcnt_h;
	    spiHw->clock.clkcnt_l		= clkcnt_l;
	}

    //Configure bit order
    spiHw->ctrl.rd_bit_order           = 0;    // MSB first
    spiHw->ctrl.wr_bit_order           = 1;    // LSB first

    //Configure polarity
    spiHw->pin.ck_idle_edge            = 0;
    spiHw->user.ck_out_edge            = 0;
    spiHw->ctrl2.miso_delay_mode       = 0;

    //configure dummy bits
    spiHw->user.usr_dummy              = 0;
    spiHw->user1.usr_dummy_cyclelen    = 0;

    //Configure misc stuff
    spiHw->user.doutdin                = 0;
    spiHw->user.sio                    = 0;

    spiHw->ctrl2.setup_time            = 0;
    spiHw->user.cs_setup               = 0;
    spiHw->ctrl2.hold_time             = 0;
    spiHw->user.cs_hold                = 0;

    //Configure CS pin
    spiHw->pin.cs0_dis                 = (Cs == 0) ? 0 : 1;
    spiHw->pin.cs1_dis                 = (Cs == 1) ? 0 : 1;
    spiHw->pin.cs2_dis                 = (Cs == 2) ? 0 : 1;

    //Reset SPI peripheral
    spiHw->dma_conf.val                |= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    spiHw->dma_out_link.start          = 0;
    spiHw->dma_in_link.start           = 0;
    spiHw->dma_conf.val                &= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);
    spiHw->dma_conf.out_data_burst_en  = 1;

    //Set up QIO/DIO if needed
    spiHw->ctrl.val		&= ~(SPI_FREAD_DUAL|SPI_FREAD_QUAD|SPI_FREAD_DIO|SPI_FREAD_QIO);
    spiHw->user.val		&= ~(SPI_FWRITE_DUAL|SPI_FWRITE_QUAD|SPI_FWRITE_DIO|SPI_FWRITE_QIO);

    //DMA temporary workaround: let RX DMA work somehow to avoid the issue in ESP32 v0/v1 silicon
    spiHw->dma_in_link.addr            = 0;
    spiHw->dma_in_link.start           = 1;

    spiHw->user1.usr_addr_bitlen       = 0;
    spiHw->user2.usr_command_bitlen    = 0;
    spiHw->user.usr_addr               = 0;
    spiHw->user.usr_command            = 0;
    if(waitCycle <= 0) {
        spiHw->user.usr_dummy              = 0;
        spiHw->user1.usr_dummy_cyclelen    = 0;
    } else {
        spiHw->user.usr_dummy              = 1;
        spiHw->user1.usr_dummy_cyclelen    = (uint8_t) (waitCycle-1);
    }

    spiHw->user.usr_mosi_highpart      = 0;
    spiHw->user2.usr_command_value     = 0;
    spiHw->addr                        = 0;
    spiHw->user.usr_mosi               = 1;        // Enable MOSI
    spiHw->user.usr_miso               = 0;

    spiHw->dma_out_link.addr           = (int)(lldescs) & 0xFFFFF;

	spiHw->mosi_dlen.usr_mosi_dbitlen  = 0;		// works great! (there's no glitch in 5 hours)
    spiHw->miso_dlen.usr_miso_dbitlen  = 0;

    // Set circular mode
    //      https://www.esp32.com/viewtopic.php?f=2&t=4011#p18107
    //      > yes, in SPI DMA mode, SPI will alway transmit and receive
    //      > data when you set the SPI_DMA_CONTINUE(BIT16) of SPI_DMA_CONF_REG.
    spiHw->dma_conf.dma_continue       = 1;

    return ESP_OK;
}


esp_err_t myspi_release_circular_buffer(
      const spi_host_device_t   spiHostDevice
    , const int                 dma_chan
    , const gpio_num_t          mosi_gpio_num
) {
	spi_dev_t* const spiHw = myspi_get_hw_for_host(spiHostDevice);

	spiHw->dma_conf.dma_continue	= 0;
	spiHw->dma_out_link.start		= 0;
	spiHw->cmd.usr					= 0;

	// TODO : Reset GPIO Matrix

	spicommon_dma_chan_free(dma_chan);
	spicommon_periph_free(spiHostDevice);
	return ESP_OK;
}
