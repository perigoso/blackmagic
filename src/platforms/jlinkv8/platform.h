/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_jlinkv8.h"

#define PLATFORM_HAS_POWER_SWITCH

#ifdef ENABLE_DEBUG
# define PLATFORM_HAS_DEBUG
#endif

#define PLATFORM_IDENT   "(JLINK-V8) "

/* Hardware definitions... */
#define TDI_PORT	0
#define TMS_PORT	0
#define TCK_PORT	0
#define TDO_PORT	0
#define TDI_PIN		0
#define TMS_PIN		0
#define TCK_PIN		0
#define TDO_PIN		0

#define SWDIO_PORT 	0
#define SWCLK_PORT 	0
#define SWDIO_PIN	0
#define SWCLK_PIN	0

#define SRST_PORT	0
#define SRST_PIN_V1	0
#define SRST_PIN_V2	0

#define LED_PORT	0
/* Use PC14 for a "dummy" uart led. So we can observere at least with scope*/
#define LED_PORT_UART	0
#define LED_UART	0

// #define NUM_TRACE_PACKETS		(128)		/* This is an 8K buffer */
// #define TRACESWO_PROTOCOL		2			/* 1 = Manchester, 2 = NRZ / async */

# define SWD_CR   GPIO_CRH(SWDIO_PORT)
# define SWD_CR_MULT (1 << ((14 - 8) << 2))

#define TMS_SET_MODE() \
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_2_MHZ, \
	              GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
#define SWDIO_MODE_FLOAT() 	do { \
	uint32_t cr = SWD_CR; \
	cr  &= ~(0xf * SWD_CR_MULT); \
	cr  |=  (0x4 * SWD_CR_MULT); \
	SWD_CR = cr; \
} while(0)
#define SWDIO_MODE_DRIVE() 	do { \
	uint32_t cr = SWD_CR; \
	cr  &= ~(0xf * SWD_CR_MULT); \
	cr  |=  (0x1 * SWD_CR_MULT); \
	SWD_CR = cr; \
} while(0)
#define UART_PIN_SETUP() do { \
	gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_50_MHZ, \
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN); \
	gpio_set_mode(USBUSART_PORT, GPIO_MODE_INPUT, \
				  GPIO_CNF_INPUT_PULL_UPDOWN, USBUSART_RX_PIN); \
	gpio_set(USBUSART_PORT, USBUSART_RX_PIN); \
} while(0)

/* Interrupt priorities.  Low numbers are high priority. */

/* On F103, only USART1 is on AHB2 and can reach 4.5 MBaud at 72 MHz.*/
#define SWO_UART				USART1
#define SWO_UART_DR				USART1_DR
#define SWO_UART_CLK			RCC_USART1
#define SWO_UART_PORT			GPIOA
#define SWO_UART_RX_PIN			GPIO10

/* This DMA channel is set by the USART in use */
#define SWO_DMA_BUS				DMA1
#define SWO_DMA_CLK				RCC_DMA1
#define SWO_DMA_CHAN			DMA_CHANNEL5
#define SWO_DMA_IRQ				NVIC_DMA1_CHANNEL5_IRQ
#define SWO_DMA_ISR(x)			dma1_channel5_isr(x)

extern uint16_t led_idle_run;
#define LED_IDLE_RUN            led_idle_run
#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, led_idle_run, state);}
#define SET_ERROR_STATE(x)

extern uint32_t detect_rev(void);

/*
 * Use newlib provided integer only stdio functions
 */

/* sscanf */
#ifdef sscanf
#undef sscanf
#define sscanf siscanf
#else
#define sscanf siscanf
#endif
/* sprintf */
#ifdef sprintf
#undef sprintf
#define sprintf siprintf
#else
#define sprintf siprintf
#endif
/* vasprintf */
#ifdef vasprintf
#undef vasprintf
#define vasprintf vasiprintf
#else
#define vasprintf vasiprintf
#endif
/* snprintf */
#ifdef snprintf
#undef snprintf
#define snprintf sniprintf
#else
#define snprintf sniprintf
#endif


#endif
