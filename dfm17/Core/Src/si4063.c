/**
  ******************************************************************************
  * @file    init.c
  * @brief   This file contains all functions for initializing hardware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Derek Rowland <gx1400@gmail.com>
  * All rights reserved.
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  * Leveraged Project
  *
  * - thasti/utrak
  *   - https://github.com/thasti/utrak
  *   - Stefan Biereigel
  *   - Modified 2023 in si4063.c and si4063.h
  ******************************************************************************
  */

#include "si4063.h"
#include "spi.h"
#include "tim.h"

void STABBY_setModemOOK(void) {
    // stuff from Si4464_Init:

    // Set transmitter GPIOs
    uint8_t gpio_pin_cfg_command[] = {
        0x13,   // Command type = GPIO settings
        0x00,   // GPIO0        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
        0x23,   // GPIO1        0 - PULL_CTL[1bit] - GPIO_MODE[6bit] // used to be 0x23 wooo
        0x00,   // GPIO2        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
        0x00,   // GPIO3        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
        0x00,   // NIRQ
        0x00,   // SDO // woo
        0x00    // GEN_CONFIG
    };

    STABBY_Si4464_write(gpio_pin_cfg_command, 8);
    HAL_Delay(25);
    //si4060_set_property_8(PROP_GLOBAL,GLOBAL_XO_TUNE,0x62);

	// Set FIFO empty interrupt threshold (32 byte)
	uint8_t set_fifo_irq[] = {0x11, 0x12, 0x01, 0x0B, 0x20};
	STABBY_Si4464_write(set_fifo_irq, 5);

	// Set FIFO to 129 byte
	uint8_t set_129byte[] = {0x11, 0x00, 0x01, 0x03, 0x10};
	STABBY_Si4464_write(set_129byte, 5);

	// Reset FIFO
	uint8_t reset_fifo[] = {0x15, 0x01};
	STABBY_Si4464_write(reset_fifo, 2);
	uint8_t unreset_fifo[] = {0x15, 0x00};
	STABBY_Si4464_write(unreset_fifo, 2);

    // Now OOK configuration stuff from setModemOOK:
	uint8_t speed_wpm = 40; // set speed to 20 wpm
    uint16_t speed = speed_wpm * 5 / 6;
    uint8_t setup_data_rate[] = {0x11, 0x20, 0x03, 0x03, 0x00, 0x00, (uint8_t)speed};
    STABBY_Si4464_write(setup_data_rate, 7);

    // Use 2FSK from FIFO (PH)
    uint8_t use_ook[] = {0x11, 0x20, 0x01, 0x00, 0x01};
    STABBY_Si4464_write(use_ook, 5);
}

void STABBY_setModemAFSK(void) {
	// stuff from: si4060_set_aprs_params_TESTING();
	/*
	si4060_set_property_8(PROP_MODEM,
			MODEM_CLKGEN_BAND,
			SY_SEL_1 | FVCO_DIV_10);
	// setup frequency deviation offset
	si4060_set_property_16(PROP_MODEM,
			MODEM_FREQ_OFFSET,
			0);
	// setup frequency deviation
	si4060_set_property_24(PROP_MODEM,
			MODEM_FREQ_DEV,
			(uint16_t)(2*FDEV_APRS_DFM));
	// set up the integer divider
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)FDIV_INTE_DFM);
	// set up the fractional divider
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)FDEV_DFM);
			*/

	// stuff from pecan Si4464_init:

    // Set transmitter GPIOs
    uint8_t gpio_pin_cfg_command[] = {
        0x13,   // Command type = GPIO settings
        0x00,   // GPIO0        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
        0x23,   // GPIO1        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
        0x00,   // GPIO2        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
        0x00,   // GPIO3        0 - PULL_CTL[1bit] - GPIO_MODE[6bit]
        0x00,   // NIRQ
        0x00,   // SDO
        0x00    // GEN_CONFIG
    };
    STABBY_Si4464_write(gpio_pin_cfg_command, 8);
    HAL_Delay(25);

	// Set FIFO empty interrupt threshold (32 byte)
	uint8_t set_fifo_irq[] = {0x11, 0x12, 0x01, 0x0B, 0x20};
	STABBY_Si4464_write(set_fifo_irq, 5);

	// Set FIFO to 129 byte
	uint8_t set_129byte[] = {0x11, 0x00, 0x01, 0x03, 0x10};
	STABBY_Si4464_write(set_129byte, 5);

	// Reset FIFO
	uint8_t reset_fifo[] = {0x15, 0x01};
	STABBY_Si4464_write(reset_fifo, 2);
	uint8_t unreset_fifo[] = {0x15, 0x00};
	STABBY_Si4464_write(unreset_fifo, 2);

	// now stuff from pecan pico 9 initAFSK:
	// Setup the NCO data rate for APRS
	uint8_t setup_data_rate[] = {0x11, 0x20, 0x03, 0x03, 0x00, 0x33, 0x90};
	STABBY_Si4464_write(setup_data_rate, 7);

	// Use 2GFSK from FIFO (PH)
	uint8_t use_2gfsk[] = {0x11, 0x20, 0x01, 0x00, 0x03};
	STABBY_Si4464_write(use_2gfsk, 5);

	// Set AFSK filter
	uint8_t coeff[] = {0x81, 0x9f, 0xc4, 0xee, 0x18, 0x3e, 0x5c, 0x70, 0x76};
	uint8_t i;
	for(i=0; i<sizeof(coeff); i++) {
		uint8_t msg[] = {0x11, 0x20, 0x01, 0x17-i, coeff[i]};
		STABBY_Si4464_write(msg, 5);
	}

	// transmit LSB first
	uint8_t use_lsb_first[] = {0x11, 0x12, 0x01, 0x06, 0x01};
	STABBY_Si4464_write(use_lsb_first, 5);
}

void STABBY_Si4464_writeFIFO(uint8_t *msg, uint8_t size) {
	uint8_t write_fifo[size+1];
	write_fifo[0] = 0x66;
    // void * memcpy ( void * destination, const void * source, size_t num );
	memcpy(&write_fifo[1], msg, size);
	STABBY_Si4464_write(write_fifo, size+1);
}

uint8_t STABBY_Si4464_freeFIFO(void) {
    // FIXME: is this acurate?
	uint8_t fifo_info[2] = {0x15, 0x00};
	uint8_t rxData[4];
	STABBY_Si4464_read(fifo_info, 2, rxData, 4);
	return rxData[3];
}

void si4060_freq_aprs_dfm17(void) {
	si4060_set_aprs_params_TESTING();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)FDIV_INTE_DFM);
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)FDEV_DFM);
}

/*
 * si4060_reset
 *
 * cleanly does the POR as specified in datasheet
 */
void si4060_reset(void) {
	si4060_shutdown();
	si4060_wakeup();
}

/*
 * si4060_shutdown
 *
 * makes the Si4060 go to shutdown state.
 * all register content is lost.
 */
void si4060_shutdown(void) {
	// SDN is PC3
	GPIOC->BSRR = (1U << +3);
	/* wait 10us */
	//__delay_cycles(50000);
	delay_us(10);
}

/*
 * si4060_wakeup
 *
 * wakes up the Si4060 from shutdown state.
 * si4060_power_up and si4060_setup have to be called afterwards
 */
void si4060_wakeup(void) {
	// SDN is PC3
	GPIOC->BSRR = (1U << (16+3));
	/* wait 20ms */
	//__delay_cycles(50000);
	delay_us(50);
	si4060_get_cts(0);
}

/*
 * si4060_part_info
 *
 * gets the PART_ID from the Si4060
 * this can be used to check for successful communication.
 * as the SPI bus returns 0xFF (MISO=high) when no slave is connected,
 * reading CTS can not verify communication to the Si4060.
 *
 * returns:	the PART_ID - should be 0x4060
 */
uint16_t si4060_part_info(void) {
	uint16_t temp;

	si4060_get_cts(0);
	temp = 0;
	spi_select();
	spi_write(CMD_PART_INFO);
	spi_deselect();
	si4060_get_cts(1);
	spi_read();	 	/* ignore CHIPREV */
	temp = spi_read(); 	/* read PART[0] */
	temp = temp << 8;
	temp |= spi_read(); 	/* read PART[1] */
	spi_deselect();
	return temp;
}

/*
 * si4060_get_cts
 *
 * waits for a CTS from the Si4060, includes a "time out"
 *
 * read_response: do not deselect the slave, if you want to read from it afterwards
 */
uint8_t si4060_get_cts(uint8_t read_response) {
	uint8_t temp = 0;
	uint8_t timeout = 0;
	if (!read_response) {
		while (si4060_read_cmd_buf(1) != 0xff && timeout < SI_TIMEOUT) {
			timeout++;
			delay_us(5);
		}
	} else {
		while (temp != 0xff && timeout < SI_TIMEOUT) {
			timeout++;
			temp = si4060_read_cmd_buf(0);
			if (temp != 0xff) {
				spi_deselect();
			}
		}
	}
	//printf("timeout count: %d\r\n", timeout);

	// todo add error return if timeout is reached
	return 0;

}

/*
 * si4060_read_cmd_buf
 *
 * reads the Si4060 command buffer from via SPI
 *
 * deselect: 	whether to deselect the slave after reading the response or not.
 * 		any command reading subsequent bytes after the CTS should use this
 * 		function to get CTS and continue doing spi_read afterwards
 * 		and finally deselecting the slave.
 *
 * returns:	the value of the first command buffer byte (i.e. CTS or not)
 */
uint8_t si4060_read_cmd_buf(uint8_t deselect) {
	uint8_t ret;
	spi_select();
	spi_write(CMD_READ_CMD_BUF);
	ret = spi_read();
	if (deselect) {
		spi_deselect();
	}
	return ret;
}


/*
 * si4060_power_up
 *
 * powers up the Si4060 by issuing the POWER_UP command
 *
 * warning: 	the si4060 can lock after issuing this command if input clock
 * 		is not available for the internal RC oscillator calibration.
 */
void si4060_power_up(void) {
	/* wait for CTS */
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_POWER_UP);
	delay_us(10);
	spi_write(FUNC);
#ifdef USE_TCXO
	spi_write(TCXO);
#else
	spi_write(0);			/* TCXO if used */
#endif
	spi_write((uint8_t) (XO_FREQ >> 24));
	spi_write((uint8_t) (XO_FREQ >> 16));
	spi_write((uint8_t) (XO_FREQ >> 8));
	spi_write((uint8_t) XO_FREQ);
	spi_deselect();
	/* wait for CTS */
	si4060_get_cts(0);
}

/*
 * si4060_change_state
 *
 * changes the internal state machine state of the Si4060
 */
void si4060_change_state(uint8_t state) {
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_CHANGE_STATE);
	spi_write(state);
	spi_deselect();

}

/*
 * si4060_start_tx
 *
 * starts transmission by the Si4060.
 *
 * channel:	the channel to start transmission on
 */
void si4060_start_tx(uint8_t channel) {
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_START_TX);
	spi_write(channel);
	spi_write(START_TX_TXC_STATE_SLEEP | START_TX_RETRANSMIT_0 | START_TX_START_IMM);
	/* set length to 0 for direct mode (is this correct?) */
	spi_write(0x00);
	spi_write(0x00);
	spi_deselect();
}

void STABBY_si4060_start_tx(uint16_t size) {
	si4060_get_cts(0);
	spi_select();
	/*spi_write(CMD_START_TX);
	spi_write(channel);
	spi_write(START_TX_TXC_STATE_SLEEP | START_TX_RETRANSMIT_0 | START_TX_START_IMM);*/
	uint8_t change_state_command[] = {0x31, 0x00, 0x30, (size >> 8) & 0x1F, size & 0xFF};
	STABBY_Si4464_write(change_state_command, 5);
	spi_deselect();
}

void STABBY_setPowerLevel(int8_t level) {
    // Set the Power
    uint8_t set_pa_pwr_lvl_property_command[] = {0x11, 0x22, 0x01, 0x01, level};
    STABBY_Si4464_write(set_pa_pwr_lvl_property_command, 5);
}


/**
 * Tunes the radio and activates transmission.
 * @param frequency Transmission frequency in Hz
 * @param shift Shift of FSK in Hz
 * @param level Transmission power level in dBm
 */
int STABBY_radioTune(uint32_t frequency, uint16_t shift, int8_t level) {


	STABBY_setFrequency(frequency, shift); // Set frequency
    //setShift(shift);                // Set shift
	/*si4060_set_property_8(PROP_MODEM,
			MODEM_CLKGEN_BAND,
			SY_SEL_1 | FVCO_DIV_10);*/
    STABBY_setPowerLevel(level);           // Set power level

    //STABBY_startTx(size);
    return 1;
}


static uint32_t outdiv;
void STABBY_setFrequency(uint32_t freq, uint16_t shift) {
    // Set the output divider according to recommended ranges given in Si4464 datasheet
    uint32_t band = 0;
    if(freq < 705000000UL) {outdiv = 6;  band = 1;};
    if(freq < 525000000UL) {outdiv = 8;  band = 2;};
    if(freq < 353000000UL) {outdiv = 12; band = 3;};
    if(freq < 239000000UL) {outdiv = 16; band = 4;};
    if(freq < 177000000UL) {outdiv = 24; band = 5;};

    // Set the band parameter
    uint32_t sy_sel = 8;
    uint8_t set_band_property_command[] = {0x11, 0x20, 0x01, 0x51, (band + sy_sel)};
    STABBY_Si4464_write(set_band_property_command, 5);

    // Set the PLL parameters
    uint32_t f_pfd = 2 * RADIO_CLK / outdiv;
    uint32_t n = ((uint32_t)(freq / f_pfd)) - 1;
    float ratio = (float)freq / (float)f_pfd;
    float rest  = ratio - (float)n;

    uint32_t m = (uint32_t)(rest * 524288UL);
    uint32_t m2 = m >> 16;
    uint32_t m1 = (m - m2 * 0x10000) >> 8;
    uint32_t m0 = (m - m2 * 0x10000 - (m1 << 8));

    uint32_t channel_increment = 524288 * outdiv * shift / (2 * RADIO_CLK);
    uint8_t c1 = channel_increment / 0x100;
    uint8_t c0 = channel_increment - (0x100 * c1);

    uint8_t set_frequency_property_command[] = {0x11, 0x40, 0x04, 0x00, n, m2, m1, m0, c1, c0};
    STABBY_Si4464_write(set_frequency_property_command, 10);

    uint32_t x = ((((uint32_t)1 << 19) * outdiv * 1300.0)/(2*RADIO_CLK))*2;
    uint8_t x2 = (x >> 16) & 0xFF;
    uint8_t x1 = (x >>  8) & 0xFF;
    uint8_t x0 = (x >>  0) & 0xFF;
    uint8_t set_deviation[] = {0x11, 0x20, 0x03, 0x0a, x2, x1, x0};
    STABBY_Si4464_write(set_deviation, 7);
}


/*
 * si4060_stop_tx
 *
 * makes the Si4060 stop all transmissions by transistioning to SLEEP state
 */
void si4060_stop_tx(void) {
	si4060_change_state(STATE_SLEEP);
}

/*
 * si4060_set_offset
 *
 * sets the FSK offset inside the Si4060 PLL. As PLL FRAC- and INTE-registers can't be modified while
 * transmitting, we must modify the MODEM_FREQ_DEV or MODEM_FREQ_OFFSET registers in transmission.
 * as we won't need more deviation than 25kHz, we can use the OFFSET register and save one byte.
 *
 * offset: frequency offset from carrier frequency (PLL tuning resolution)
 *
 */
void si4060_set_offset(uint16_t offset) {
	si4060_set_property_16_nocts(PROP_MODEM, MODEM_FREQ_OFFSET, offset);
}

/*
 * si4060_nop
 *
 * implements the NOP command on the Si4060
 */
void si4060_nop(void) {
	spi_select();
	spi_write(CMD_NOP);
	spi_deselect();
	si4060_get_cts(0);
}

/*
 * si4060_set_property_8
 *
 * sets an 8 bit (1 byte) property in the Si4060
 *
 * group:	the group number of the property
 * prop:	the number (index) of the property
 * val:		the value to set
 */
void si4060_set_property_8(uint8_t group, uint8_t prop, uint8_t val) {
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_SET_PROPERTY);
	spi_write(group);
	spi_write(1);
	spi_write(prop);
	spi_write(val);
	spi_deselect();
}

void STABBY_Si4464_write(uint8_t* txData, uint32_t len) {
	si4060_get_cts(0);
	spi_select();

/*
	uint8_t index = 0;
	for (index = 0; index < len; index++) {
		spi_write(txData[index]);
	}
    */
    SpiWriteData(len, txData);
	spi_deselect();
	si4060_get_cts(0);
	//HAL_Delay(50);
	//spi_read(); /* read property */
	//spi_deselect();
  /* 
    uint8_t rxData[len];
    rxData[1] = 0x00;
    while (rxData[1] != 0xFF) {
        // Request ACK by Si4464
        uint8_t rx_ready[] = {0x44};
        // SPI transfer
        spi_select();
        spi_write(rx_ready);
        SpiReadData(3, rxData);
        spi_deselect();
    }
    */
  
}


/*
 * si4060_get_property_8
 *
 * gets an 8 bit (1 byte) property in the Si4060
 *
 * group:	the group number of the property
 * prop:	the number (index) of the property
 *
 * returns:	the value of the property
 */
uint8_t si4060_get_property_8(uint8_t group, uint8_t prop) {
	uint8_t temp = 0;
	spi_select();
	spi_write(CMD_GET_PROPERTY);
	spi_write(group);
	spi_write(1);
	spi_write(prop);
	spi_deselect();
	si4060_get_cts(1);
	temp = spi_read(); /* read property */
	spi_deselect();
	return temp;
}

/*
 * si4060_set_property_16
 *
 * sets an 16 bit (2 byte) property in the Si4060
 *
 * group:	the group number of the property
 * prop:	the number (index) of the property
 * val:		the value to set
 */
void si4060_set_property_16(uint8_t group, uint8_t prop, uint16_t val) {
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_SET_PROPERTY);
	spi_write(group);
	spi_write(2);
	spi_write(prop);
	spi_write(val >> 8);
	spi_write(val);
	spi_deselect();
}

/*
 * si4060_set_property_16_nocts
 *
 * sets an 16 bit (2 byte) property in the Si4060
 * does not check for CTS from the Si4060
 *
 * group:	the group number of the property
 * prop:	the number (index) of the property
 * val:		the value to set
 */
void si4060_set_property_16_nocts(uint8_t group, uint8_t prop, uint16_t val) {
	spi_select();
	spi_write(CMD_SET_PROPERTY);
	spi_write(group);
	spi_write(2);
	spi_write(prop);
	spi_write(val >> 8);
	spi_write(val);
	spi_deselect();
}

/*
 * si4060_set_property_24
 *
 * sets an 24 bit (3 byte) property in the Si4060
 *
 * group:	the group number of the property
 * prop:	the number (index) of the property
 * val:		the value to set
 */
void si4060_set_property_24(uint8_t group, uint8_t prop, uint32_t val) {
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_SET_PROPERTY);
	spi_write(group);
	spi_write(3);
	spi_write(prop);
	spi_write(val >> 16);
	spi_write(val >> 8);
	spi_write(val);
	spi_deselect();
}

/*
 * si4060_set_property_32
 *
 * sets an 32 bit (4 byte) property in the Si4060
 *
 * group:	the group number of the property
 * prop:	the number (index) of the property
 * val:		the value to set
 */
void si4060_set_property_32(uint8_t group, uint8_t prop, uint32_t val) {
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_SET_PROPERTY);
	spi_write(group);
	spi_write(4);
	spi_write(prop);
	spi_write(val >> 24);
	spi_write(val >> 16);
	spi_write(val >> 8);
	spi_write(val);
	spi_deselect();
}

/*
 * si4060_setup
 *
 * initializes the Si4060 by setting all neccesary internal registers.
 * has to be called after si4060_power_up.
 *
 * mod_type:	the type of modulation to use, use the MODEM_MOD_TYPE values (MOD_TYPE_*)
 *
 * Modified for DFM17:
 * - Rearranged early functions
 * - Changes gpio pin assignment
 * - changes TXCO Global Tune value from 0x00 to 0x62 for capacitance
 */
void si4060_setup(uint8_t mod_type) {
	/* set up GPIOs */
	si4060_gpio_pin_cfg(GPIO_MODE_TX_DATA_CLK,
			GPIO_MODE_EN_PA,
			GPIO_MODE_DIV_CLK,
			GPIO_MODE_INPUTPIN,
			DRV_STRENGTH_HIGH);

#ifdef	USE_TCXO
	si4060_set_property_8(PROP_GLOBAL,
			GLOBAL_XO_TUNE,
			0x62);
#endif

	/* Global clock config */
	si4060_set_property_8(PROP_GLOBAL,
			GLOBAL_CLK_CFG,
			0x60);

	/* set high performance mode */
	si4060_set_property_8(PROP_GLOBAL,
			GLOBAL_CONFIG,
			GLOBAL_RESERVED | POWER_MODE_HIGH_PERF | SEQUENCER_MODE_FAST);

	/* PROP_INT_CTL */
	si4060_set_property_8(PROP_GLOBAL,
			PROP_INT_CTL, 0x00);

	si4060_set_property_32(PROP_GLOBAL,
			PROP_FRR_CTL, 0x00000000);

	/* disable preamble */
	si4060_set_property_8(PROP_PREAMBLE,
			PREAMBLE_TX_LENGTH,
			0);

	/* do not transmit sync word */
	si4060_set_property_8(PROP_SYNC,
			SYNC_CONFIG,
			SYNC_NO_XMIT);

	/* setup the NCO modulo and oversampling mode */
	si4060_set_property_32(PROP_MODEM,
			MODEM_TX_NCO_MOD,
			MOD_TX_OSR_10 | (XO_FREQ / 10));



	/* set up the PA duty cycle */
	si4060_set_property_8(PROP_PA,
			PA_BIAS_CLKDUTY,
			PA_BIAS_CLKDUTY_SIN_25);
	//si4060_set_filter();
}

/*
 * si4060_set_filter
 *
 * writes the bandpass filter coefficients into the Si4060. these realize a low pass filter for
 * harmonics of the square wave modulation waveform and a high pass filter for the
 * APRS preemphasis.
 *
 */
void si4060_set_filter(void) {
	//uint8_t coeff[9] = {0x1d, 0xe5, 0xb8, 0xaa, 0xc0, 0xf5, 0x36, 0x6b, 0x7f};	// 6dB@1200 Hz, 2400 Hz
	//uint8_t coeff[9] = {0x07, 0xde, 0xbf, 0xb9, 0xd4, 0x05, 0x40, 0x6d, 0x7f};	// 3db@1200 Hz, 2400 Hz
	//uint8_t coeff[9] = {0xfa, 0xe5, 0xd8, 0xde, 0xf8, 0x21, 0x4f, 0x71, 0x7f};	// LP only, 2400 Hz
	//uint8_t coeff[9] = {0xd9, 0xf1, 0x0c, 0x29, 0x44, 0x5d, 0x70, 0x7c, 0x7f}; 	// LP only, 4800 Hz
	//uint8_t coeff[9] = {0xd5, 0xe9, 0x03, 0x20, 0x3d, 0x58, 0x6d, 0x7a, 0x7f}; 	// LP only, 4400 Hz
	uint8_t coeff[9] = {0x81, 0x9f, 0xc4, 0xee, 0x18, 0x3e, 0x5c, 0x70, 0x76};	// 6dB@1200Hz, 4400 Hz (bad stopband)

	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_8,
			coeff[8]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_7,
			coeff[7]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_6,
			coeff[6]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_5,
			coeff[5]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_4,
			coeff[4]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_3,
			coeff[3]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_2,
			coeff[2]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_1,
			coeff[1]);
	si4060_set_property_8(PROP_MODEM,
			MODEM_TX_FILTER_COEFF_0,
			coeff[0]);
}

/*
 * si4060_gpio_pin_cfg
 *
 * configures the GPIOs on the Si4060
 * see the GPIO_*-defines for reference
 *
 * gpio(0..3):	setting flags for respective GPIOs
 * drvstrength:	the driver strength
 */
void si4060_gpio_pin_cfg(uint8_t gpio0, uint8_t gpio1,
		uint8_t gpio2, uint8_t gpio3, uint8_t drvstrength) {
	si4060_get_cts(0);
	spi_select();
	spi_write(CMD_GPIO_PIN_CFG);
	spi_write(gpio0);
	spi_write(gpio1);
	spi_write(gpio2);
	spi_write(gpio3);
	spi_write(NIRQ_MODE_DONOTHING);
	spi_write(GPIO_MODE_SDO);
	spi_write(drvstrength);
	spi_deselect();
}

void si4060_set_aprs_params(void) {
	/* use 2GFSK from async GPIO0 */
	si4060_set_property_8(PROP_MODEM,
			MODEM_MOD_TYPE,
			MOD_TYPE_2GFSK | MOD_SOURCE_DIRECT | MOD_GPIO_3 | MOD_DIRECT_MODE_SYNC);
	/* setup divider to 24 (for 2m amateur radio band) */
	si4060_set_property_8(PROP_MODEM,
			MODEM_CLKGEN_BAND,
			SY_SEL_1 | FVCO_DIV_24);
	/* setup frequency deviation offset */
	si4060_set_property_16(PROP_MODEM,
			MODEM_FREQ_OFFSET,
			0);
	/* setup frequency deviation */
	si4060_set_property_24(PROP_MODEM,
			MODEM_FREQ_DEV,
			(uint16_t)(2*FDEV_APRS));
}


void si4060_set_aprs_params_TESTING(void) {
	/* use 2GFSK from async GPIO0 */
	si4060_set_property_8(PROP_MODEM,
			MODEM_MOD_TYPE,
			MOD_TYPE_2GFSK | MOD_SOURCE_DIRECT | MOD_GPIO_3 | MOD_DIRECT_MODE_SYNC);
	/* setup divider to 10 (for DFMtesting at 400Mhz) */
	si4060_set_property_8(PROP_MODEM,
			MODEM_CLKGEN_BAND,
			SY_SEL_1 | FVCO_DIV_10);
	/* setup frequency deviation offset */
	si4060_set_property_16(PROP_MODEM,
			MODEM_FREQ_OFFSET,
			0);
	/* setup frequency deviation */
	si4060_set_property_24(PROP_MODEM,
			MODEM_FREQ_DEV,
			(uint16_t)(2*FDEV_APRS_DFM));
}



void si4060_freq_aprs_reg1(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(EU)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(EU)));
}

void si4060_freq_aprs_reg2(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(US)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(US)));
}

void si4060_freq_aprs_cn(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(CN)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(CN)));
}

void si4060_freq_aprs_jp(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(JP)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(JP)));
}

void si4060_freq_aprs_thai(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(THAI)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(THAI)));
}

void si4060_freq_aprs_nz(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(NZ)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(NZ)));
}

void si4060_freq_aprs_aus(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(AUS)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(AUS)));
}

void si4060_freq_aprs_brazil(void) {
	si4060_set_aprs_params();
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(BRAZIL)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(BRAZIL)));
}

void si4060_freq_2m_rtty(void) {
	/* use 2FSK from async GPIO0 */
	si4060_set_property_8(PROP_MODEM,
			MODEM_MOD_TYPE,
			MOD_TYPE_2FSK | MOD_SOURCE_DIRECT | MOD_GPIO_3 | MOD_DIRECT_MODE_ASYNC);
	/* setup divider to 24 (for 2m-band) */
	si4060_set_property_8(PROP_MODEM,
			MODEM_CLKGEN_BAND,
			SY_SEL_1 | FVCO_DIV_24);
	/* set up the integer divider */
	si4060_set_property_8(PROP_FREQ_CONTROL,
			FREQ_CONTROL_INTE,
			(uint8_t)(FDIV_INTE_2M(RTTY)));
	/* set up the fractional divider */
	si4060_set_property_24(PROP_FREQ_CONTROL,
			FREQ_CONTROL_FRAC,
			(uint32_t)(FDIV_FRAC_2M(RTTY)));
	/* setup frequency deviation offset */
	si4060_set_property_16(PROP_MODEM,
			MODEM_FREQ_OFFSET,
			0);
	/* setup frequency deviation */
	si4060_set_property_24(PROP_MODEM,
			MODEM_FREQ_DEV,
			(uint16_t)(FDEV_RTTY));
}



void __delay_cycles(uint32_t delay) {
	for (uint32_t x = 0; x < delay; x++);
}
