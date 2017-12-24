#include <soc/spi_reg.h>
#include <soc/dport_reg.h>
#include "my_spi.h"

spi_dev_t *myspi_get_hw_for_host(
    spi_host_device_t host
) {
    spi_dev_t* hw = NULL;
    switch(host) {
    case SPI_HOST:  hw = &SPI1; break;
    case HSPI_HOST: hw = &SPI2; break;
    case VSPI_HOST: hw = &SPI3; break;
    default:        hw = NULL;  break;
    }
    return hw;
}


static uint8_t getSpidOutFromHost(
    spi_host_device_t host
) {
    uint8_t o = (uint8_t) -1;
    switch(host) {
    case SPI_HOST:  o = SPID_OUT_IDX;   break;
    case HSPI_HOST: o = HSPID_OUT_IDX;  break;
    case VSPI_HOST: o = VSPID_OUT_IDX;  break;
    default:                            break;
    }
    return o;
}


static uint8_t getSpidInFromHost(
    spi_host_device_t host
) {
    uint8_t o = (uint8_t) -1;
    switch(host) {
    case SPI_HOST:  o = SPID_IN_IDX;    break;
    case HSPI_HOST: o = HSPID_IN_IDX;   break;
    case VSPI_HOST: o = VSPID_IN_IDX;   break;
    default:                            break;
    }
    return o;
}


esp_err_t myspi_prepare_circular_buffer(
      const spi_host_device_t   spiHostDevice
    , const int                 dma_chan
    , const lldesc_t*           lldescs
    , const int                 bitsPerLoop
    , const double              dmaClockSpeedInHz
    , const gpio_num_t          mosi_gpio_num
    , const int                 waitCycle
) {
    const bool spi_chan_claimed = spicommon_periph_claim(spiHostDevice);
    if(! spi_chan_claimed) {
        return MY_ESP_ERR_SPI_HOST_ALREADY_IN_USE;
    }

    const bool dma_chan_claimed = spicommon_dma_chan_claim(dma_chan);
    if(! dma_chan_claimed) {
        spicommon_periph_free(spiHostDevice);
        return MY_ESP_ERR_SPI_DMA_ALREADY_IN_USE;
    }

    spi_dev_t* const hw = myspi_get_hw_for_host(spiHostDevice);
    const int Cs = 0;
	const int CsMask = 1 << Cs;

    //Use GPIO
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[mosi_gpio_num], PIN_FUNC_GPIO);
    gpio_set_direction(mosi_gpio_num, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(mosi_gpio_num, getSpidOutFromHost(spiHostDevice), false, false);
    gpio_matrix_in(mosi_gpio_num, getSpidInFromHost(spiHostDevice), false);

    //Select DMA channel.
    DPORT_SET_PERI_REG_BITS(
          DPORT_SPI_DMA_CHAN_SEL_REG
        , 3
        , dma_chan
        , (spiHostDevice * 2)
    );

    //Reset DMA
    hw->dma_conf.val        		|= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    hw->dma_out_link.start  		= 0;
    hw->dma_in_link.start   		= 0;
    hw->dma_conf.val        		&= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    //Reset timing
    hw->ctrl2.val           		= 0;

    //Disable unneeded ints
    hw->slave.rd_buf_done   		= 0;
    hw->slave.wr_buf_done   		= 0;
    hw->slave.rd_sta_done   		= 0;
    hw->slave.wr_sta_done   		= 0;
    hw->slave.rd_buf_inten  		= 0;
    hw->slave.wr_buf_inten  		= 0;
    hw->slave.rd_sta_inten  		= 0;
    hw->slave.wr_sta_inten  		= 0;
    hw->slave.trans_inten   		= 0;
    hw->slave.trans_done    		= 0;

    //Set CS pin, CS options
	hw->pin.master_ck_sel			&= ~CsMask;
	hw->pin.master_cs_pol			&= ~CsMask;

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

		hw->clock.clk_equ_sysclk	= 0;
	    hw->clock.clkcnt_n			= clkcnt_n;
	    hw->clock.clkdiv_pre		= clkdiv_pre;
		hw->clock.clkcnt_h			= clkcnt_h;
	    hw->clock.clkcnt_l			= clkcnt_l;
	}

    //Configure bit order
    hw->ctrl.rd_bit_order           = 0;    // MSB first
    hw->ctrl.wr_bit_order           = 0;    // MSB first

    //Configure polarity
    hw->pin.ck_idle_edge            = 0;
    hw->user.ck_out_edge            = 0;
    hw->ctrl2.miso_delay_mode       = 0;

    //configure dummy bits
    hw->user.usr_dummy              = 0;
    hw->user1.usr_dummy_cyclelen    = 0;

    //Configure misc stuff
    hw->user.doutdin                = 0;
    hw->user.sio                    = 0;

    hw->ctrl2.setup_time            = 0;
    hw->user.cs_setup               = 0;
    hw->ctrl2.hold_time             = 0;
    hw->user.cs_hold                = 0;

    //Configure CS pin
    hw->pin.cs0_dis                 = (Cs == 0) ? 0 : 1;
    hw->pin.cs1_dis                 = (Cs == 1) ? 0 : 1;
    hw->pin.cs2_dis                 = (Cs == 2) ? 0 : 1;

    //Reset SPI peripheral
    hw->dma_conf.val                |= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    hw->dma_out_link.start          = 0;
    hw->dma_in_link.start           = 0;
    hw->dma_conf.val                &= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);
    hw->dma_conf.out_data_burst_en  = 1;

    //Set up QIO/DIO if needed
    hw->ctrl.val					&= ~(SPI_FREAD_DUAL|SPI_FREAD_QUAD|SPI_FREAD_DIO|SPI_FREAD_QIO);
    hw->user.val					&= ~(SPI_FWRITE_DUAL|SPI_FWRITE_QUAD|SPI_FWRITE_DIO|SPI_FWRITE_QIO);

    //DMA temporary workaround: let RX DMA work somehow to avoid the issue in ESP32 v0/v1 silicon
    hw->dma_in_link.addr            = 0;
    hw->dma_in_link.start           = 1;

    hw->user1.usr_addr_bitlen       = 0;
    hw->user2.usr_command_bitlen    = 0;
    hw->user.usr_addr               = 0;
    hw->user.usr_command            = 0;
    if(waitCycle <= 0) {
        hw->user.usr_dummy              = 0;
        hw->user1.usr_dummy_cyclelen    = 0;
    } else {
        hw->user.usr_dummy              = 1;
        hw->user1.usr_dummy_cyclelen    = (uint8_t) (waitCycle-1);
    }

    hw->user.usr_mosi_highpart      = 0;
    hw->user2.usr_command_value     = 0;
    hw->addr                        = 0;
    hw->user.usr_mosi               = 1;        // Enable MOSI
    hw->user.usr_miso               = 0;

    hw->dma_out_link.addr           = (int)(lldescs) & 0xFFFFF;

	hw->mosi_dlen.usr_mosi_dbitlen  = 0;		// works great! (there's no glitch in 5 hours)
    hw->miso_dlen.usr_miso_dbitlen  = 0;

    // Set circular mode
    //      https://www.esp32.com/viewtopic.php?f=2&t=4011#p18107
    //      > yes, in SPI DMA mode, SPI will alway transmit and receive
    //      > data when you set the SPI_DMA_CONTINUE(BIT16) of SPI_DMA_CONF_REG.
    hw->dma_conf.dma_continue       = 1;

    return ESP_OK;
}
