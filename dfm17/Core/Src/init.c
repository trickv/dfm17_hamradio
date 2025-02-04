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
  */

#include "init.h"
#include "GNSS.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "si4063.h"


extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;
extern GNSS_StateHandle GNSS_Handle;
extern uint8_t txDone;
extern uint8_t rxDone;

void initHw(void) {
	// initialize STM hardware
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_TIM6_Init();
	MX_TIM17_Init();
	delay_us(50);

	//initialize radio
	initRadio();

	// initialize GPS chip
	printf("Starting ublox...\r\n");
	GNSS_Init(&GNSS_Handle, &huart2, &txDone, &rxDone);
	HAL_Delay(1000);
	GNSS_LoadConfig(&GNSS_Handle);

	//after GPS is initialized, then start GPS update tick timer
	startGpsTickTimer();
}

void initRadio() {
	SpiEnable();

	//restart radio
	printf("wake up radio...\r\n");
	si4060_wakeup();
	HAL_Delay(10);
	printf("reset radio...\r\n");
	si4060_reset();

	printf("check radio info...\r\n");
	uint16_t i;
	i = si4060_part_info();
	printf("Radio info: %04X\r\n",i);

	if(i != 0x4063) {
		printf("ERROR: Incorrect radio, not a 4063!\r\n");
	}

	si4060_power_up(); 		//power up radio
	si4060_change_state(STATE_TX);
}
