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

/* This file implements the platform specific functions for the native implementation. */

#include "general.h"
#include "platform.h"
#include "usb.h"
#include "aux_serial.h"
#include "morse.h"
#include "bitbang_jtag.h"

#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>

static void adc_init(void);
static void setup_vbus_irq(void);

#define TPWR_SOFT_START_STEPS 64U

/*
 * Starting with hardware version 4 we are storing the hardware version in the
 * flash option user Data1 byte.
 * The hardware version 4 was the transition version that had it's hardware
 * pins strapped to 3 but contains version 4 in the Data1 byte.
 * The hardware 4 is backward compatible with V3 but provides the new jumper
 * connecting STRACE target pin to the UART1 pin.
 * Hardware version 5 does not have the physically strapped version encoding
 * any more and the hardware version has to be read out of the option bytes.
 * This means that older firmware versions that don't do the detection won't
 * work on the newer hardware.
 */
#define BMP_HWVERSION_BYTE FLASH_OPTION_BYTE_2

int hwversion = -1;

/*
 * Pins PB[7:5] are used to detect hardware revision.
 * User option byte Data1 is used starting with hardware revision 4.
 * Pin -  OByte - Rev - Description
 * 000 - 0xffff -   0 - Original production build.
 * 001 - 0xffff -   1 - Mini production build.
 * 010 - 0xffff -   2 - Mini V2.0e and later.
 * 011 - 0xffff -   3 - Mini V2.1a and later.
 * 011 - 0xfb04 -   4 - Mini V2.1d and later.
 * xxx - 0xfb05 -   5 - Mini V2.2a and later.
 * xxx - 0xfb06 -   6 - Mini V2.3a and later.
 *
 * This function will return -2 if the version number does not make sense.
 * This can happen when the Data1 byte contains "garbage". For example a
 * hardware revision that is <4 or the high byte is not the binary inverse of
 * the lower byte.
 * Note: The high byte of the Data1 option byte should always be the binary
 * inverse of the lower byte unless the byte is not set, then all bits in both
 * high and low byte are 0xff.
 */
static int platform_hwversion_init(void)
{
	uint16_t hwversion_pins = GPIO7 | GPIO6 | GPIO5;
	uint16_t unused_pins = hwversion_pins ^ 0xffffU;

	/* Check if the hwversion is set in the user option byte. */
	if (hwversion == -1) {
		if (BMP_HWVERSION_BYTE != 0xffffU && BMP_HWVERSION_BYTE != 0x00ffU) {
			/* Check if the data is valid. When valid it should only have values 4 and higher. */
			if ((BMP_HWVERSION_BYTE >> 8U) != (~BMP_HWVERSION_BYTE & 0xffU) || (BMP_HWVERSION_BYTE & 0xffU) < 4)
				return -2;
			hwversion = BMP_HWVERSION_BYTE & 0xffU;
		}
	}

	/* If the hwversion is not set in option bytes check the hw pin strapping. */
	if (hwversion == -1) {
		/* Configure the hardware version pins as input pull-up/down */
		gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, hwversion_pins);

		/* Enable the weak pull up. */
		gpio_set(GPIOB, hwversion_pins);

		/* Wait a little to make sure the pull up is in effect... */
		for (volatile size_t i = 0; i < 100U; ++i)
			continue;

		/*
		 * Get all pins that are pulled low in hardware.
		 * This also sets all the "unused" pins to 1.
		 */
		uint16_t pins_negative = gpio_get(GPIOB, hwversion_pins) | unused_pins;

		/* Enable the weak pull down. */
		gpio_clear(GPIOB, hwversion_pins);

		/* Wait a little to make sure the pull down is in effect... */
		for (volatile size_t i = 0; i < 100U; ++i)
			continue;

		/* Get all the pins that are pulled high in hardware. */
		uint16_t pins_positive = gpio_get(GPIOB, hwversion_pins);

		/*
		 * The hardware version is the ID defined by the pins that are
		 * asserted low or high by the hardware. This means that pins
		 * that are left floating are 0 and those that are either
		 * pulled high or low are 1.
		 *
		 * XXX: This currently converts `uint16_t`'s to `int`. It should not do this,
		 * it should remain unsigned at all times, but this requires changing how the invalid
		 * hardware version should be returned.
		 */
		hwversion = (((pins_positive ^ pins_negative) ^ 0xffffU) & hwversion_pins) >> 5U;
	}

	return hwversion;
}

void platform_init(void)
{
	const int hwversion = platform_hwversion_init();
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	if (hwversion >= 6)
		rcc_periph_clock_enable(RCC_GPIOC);
	if (hwversion >= 1)
		rcc_periph_clock_enable(RCC_TIM1);
	/* Make sure to power up the timer used for trace */
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_CRC);

	/* Setup GPIO ports */
	gpio_clear(USB_PU_PORT, USB_PU_PIN);
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);

	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TMS_DIR_PIN | TCK_PIN | TDI_PIN);
	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_INPUT_FLOAT, TMS_PIN);
	gpio_set_mode(JTAG_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, TDO_PIN);

	/* This needs some fixing... */
	/* Toggle required to sort out line drivers... */
	gpio_port_write(GPIOA, 0x8102);
	gpio_port_write(GPIOB, 0x2000);

	gpio_port_write(GPIOA, 0x8182);
	gpio_port_write(GPIOB, 0x2002);

	if (hwversion >= 6) {
		gpio_set_mode(TCK_DIR_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TCK_DIR_PIN);
		gpio_set_mode(TCK_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, TCK_PIN);
		gpio_clear(TCK_DIR_PORT, TCK_DIR_PIN);
	}

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_UART | LED_IDLE_RUN | LED_ERROR);

	/*
	 * Enable nRST output. Original uses a NPN to pull down, so setting the
	 * output HIGH asserts. Mini is directly connected so use open drain output
	 * and set LOW to assert.
	 */
	platform_nrst_set_val(false);
	gpio_set_mode(NRST_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		hwversion == 0 || hwversion >= 3 ? GPIO_CNF_OUTPUT_PUSHPULL : GPIO_CNF_OUTPUT_OPENDRAIN, NRST_PIN);
	/* FIXME: Gareth, Esden, what versions need this fix? */
	if (hwversion < 3)
		/*
		 * FIXME: This pin in intended to be input, but the TXS0108 fails
		 * to release the device from reset if this floats.
		 */
		gpio_set_mode(NRST_SENSE_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, NRST_SENSE_PIN);
	else {
		gpio_set(NRST_SENSE_PORT, NRST_SENSE_PIN);
		gpio_set_mode(NRST_SENSE_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, NRST_SENSE_PIN);
	}
	/*
	 * Enable internal pull-up on PWR_BR so that we don't drive
	 * TPWR locally or inadvertently supply power to the target.
	 */
	if (hwversion == 1) {
		gpio_set(PWR_BR_PORT, PWR_BR_PIN);
		gpio_set_mode(PWR_BR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, PWR_BR_PIN);
	} else if (hwversion > 1) {
		gpio_set(PWR_BR_PORT, PWR_BR_PIN);
		gpio_set_mode(PWR_BR_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, PWR_BR_PIN);
	}

	/* Configure Timer 1 Channel 3N to allow tpwr to be soft start on hw1+ */
	if (hwversion >= 1) {
		/* The pin mapping is a secondary mapping for the pin. We need to enable that. */
		gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ, AFIO_MAPR_TIM1_REMAP_PARTIAL_REMAP);
		/*
		 * Configure Timer 1 to run the the power control pin PWM and switch the timer on
		 * NB: We don't configure the pin mode here but rather we configure it to the alt-mode and back in
		 * platform_target_set_power() below due to GD32 errata involving PB2 (AUX serial LED).
		 * See §3.7.6 of the GD32F103 Compatibility Summary for details.
		 */
		timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
		/* Use PWM mode 1 so that the signal generated is low till it exceeds the set value */
		timer_set_oc3_mode(TIM1, TIM_OCM_PWM1);
		/* Mark the output active-low due to how this drives the target pin */
		timer_set_oc_polarity_low(TIM1, TIM_OC3N);
		timer_enable_oc_output(TIM1, TIM_OC3N);
		timer_set_oc_value(TIM1, TIM_OC3, 0);
		/* Make sure dead-time is switched off as this interferes with the correct waveform generation */
		timer_set_deadtime(TIM1, 0);
		/*
		 * Configure for 64 steps which also makes this output a 562.5kHz PWM signal
		 * given the lack of prescaling and being a peripheral on APB1 (36MHz)
		 */
		timer_set_period(TIM1, TPWR_SOFT_START_STEPS - 1U);
		timer_enable_break_main_output(TIM1);
		timer_continuous_mode(TIM1);
		timer_update_on_overflow(TIM1);
		timer_enable_counter(TIM1);
	}

	if (hwversion >= 5) {
		gpio_set_mode(AUX_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, AUX_SCLK | AUX_COPI);
		gpio_set_mode(AUX_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, AUX_FCS | AUX_SDCS);
		gpio_set_mode(AUX_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, AUX_CIPO);
		gpio_set(AUX_PORT, AUX_FCS | AUX_SDCS);
		/* hw6 introduced an SD Card chip select on PB6, moving the display select to PB7 */
		if (hwversion >= 6) {
			gpio_set_mode(AUX_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, AUX_DCS6);
			gpio_set(AUX_PORT, AUX_DCS6);
		}
	}

	if (hwversion > 0)
		adc_init();
	else {
		gpio_clear(GPIOB, GPIO0);
		gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
	}
	/* Set up the NVIC vector table for the firmware */
	SCB_VTOR = (uintptr_t)&vector_table; // NOLINT(clang-diagnostic-pointer-to-int-cast, performance-no-int-to-ptr)

	platform_timing_init();
	blackmagic_usb_init();

	/*
	 * On hardware version 1 and 2, UART and SWD share connector pins.
	 * Don't enable UART if we're being debugged.
	 */
	if (hwversion == 0 || hwversion >= 3 || !(SCS_DEMCR & SCS_DEMCR_TRCENA))
		aux_serial_init();

	setup_vbus_irq();

	bitbang_jtag_register(); /* Register JTAG interface */
}

int platform_hwversion(void)
{
	return hwversion;
}

void platform_nrst_set_val(bool assert)
{
	gpio_set(TMS_PORT, TMS_PIN);
	if (hwversion == 0 || hwversion >= 3)
		gpio_set_val(NRST_PORT, NRST_PIN, assert);
	else
		gpio_set_val(NRST_PORT, NRST_PIN, !assert);

	if (assert) {
		for (volatile size_t i = 0; i < 10000U; ++i)
			continue;
	}
}

bool platform_nrst_get_val(void)
{
	if (hwversion == 0)
		return gpio_get(NRST_SENSE_PORT, NRST_SENSE_PIN) == 0;
	if (hwversion >= 3)
		return gpio_get(NRST_SENSE_PORT, NRST_SENSE_PIN) != 0;
	return gpio_get(NRST_PORT, NRST_PIN) == 0;
}

bool platform_target_get_power(void)
{
	if (hwversion > 0)
		return !gpio_get(PWR_BR_PORT, PWR_BR_PIN);
	return false;
}

static inline void platform_wait_pwm_cycle()
{
	while (!timer_get_flag(TIM1, TIM_SR_UIF))
		continue;
	timer_clear_flag(TIM1, TIM_SR_UIF);
}

bool platform_target_set_power(const bool power)
{
	if (hwversion <= 0)
		return false;
	/* If we're on hw1 or newer, and are turning the power on */
	if (power) {
		/* Configure the pin to be driven by the timer */
		gpio_set_mode(PWR_BR_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, PWR_BR_PIN);
		timer_clear_flag(TIM1, TIM_SR_UIF);
		/* Wait for one PWM cycle to have taken place */
		platform_wait_pwm_cycle();
		/* Soft start power on the target */
		for (size_t step = 1U; step < TPWR_SOFT_START_STEPS; ++step) {
			/* Set the new PWM value */
			timer_set_oc_value(TIM1, TIM_OC3, step);
			/* Wait for one PWM cycle to have taken place */
			platform_wait_pwm_cycle();
		}
	}
	/* Set the pin state */
	gpio_set_val(PWR_BR_PORT, PWR_BR_PIN, !power);
	/*
	 * If we're turning power on and running hw1+, now configure the pin back over to GPIO and
	 * reset state timer for the next request
	 */
	if (power) {
		if (hwversion == 1)
			gpio_set_mode(PWR_BR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, PWR_BR_PIN);
		else
			gpio_set_mode(PWR_BR_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, PWR_BR_PIN);
		timer_set_oc_value(TIM1, TIM_OC3, 0U);
	}
	return true;
}

static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

	gpio_set_mode(TPWR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, TPWR_PIN);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
	adc_enable_temperature_sensor();
	adc_power_on(ADC1);

	/* Wait for the ADC to finish starting up */
	for (volatile size_t i = 0; i < 800000U; ++i)
		continue;

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

/* Reference voltage comes from the 3.3V regulator, this is a variable to allow for calibration */
// static uint32_t adc_reference = 3300U;

/*
 * The calculation for the voltage on TPWR for a given ADC value is as follows:
 * 
 *                adc_ref_mv
 * adc_unit_mv = ──────────── (mV per unit of ADC value)
 *               2 ^ adc_bits
 *
 *                 r1 + r2
 * inv_div_ratio = ─────── (inverse of the voltage divider ratio)
 *                   r2
 *
 * tpwr_unit_mv = adc_unit_mv * inv_div_ratio (TPWR mV per unit of ADC value)
 *  
 *                 adc_ref_mv    r1 + r2   adc_ref_mv * (r1 + r2)
 * tpwr_unit_mv = ──────────── * ─────── = ──────────────────────
 *                2 ^ adc_bits     r2       (2 ^ adc_bits) * r2
 *
 * We additionally shift the dividend by fixed point bits to account for the fractional part
 * This means that multiplying the ADC value by this value gives us the voltage in mV in fixed point format
 *
 * We can shift the result back by the fixed point bits to get an integer value in mV,
 * but note that this results in a loss of precision, and the result is rounded *down* to the previous integer,
 * to get rounding to the nearest integer we can add half the fixed point ratio to the value before the shifting
 * 
 * The result given by the (SAR) ADC is not exact, it is a possible range between the given value and the next higher value
 * We can add half the range to the value to get a more accurate result on average
 * 
 * We use 18-bit fractional because the remaining 14-bit can represent a voltage up to ~16V in 1mV steps which is more than enough 
 * 18-bit fractional gives a more accurate result at no cost, we can lose up to 1 bit of precision inherent to the calculation
 * which is a maximum absolute error of <0.001%
 */
#define VOLT_FP_BITS           18U                           /* Number of fractional bits in the fixed point value */
#define VOLT_FP_MASK           ((1U << (VOLT_FP_BITS)) - 1U) /* Mask for the fractional part of the fixed point value */
#define VOLT_FP_HALF           (1U << ((VOLT_FP_BITS)-1U))   /* Half of the fixed point ratio value */
#define VOLT_FP_TO_UINT32(val) (((val) + VOLT_FP_HALF) >> VOLT_FP_BITS) /* Round to nearest integer */

#define VOLT_ADC_RES_BITS 12U                       /* Number of resolution bits in the ADC value, 12-bit */
#define VOLT_ADC_RES      (1U << VOLT_ADC_RES_BITS) /* Number of ADC values */
#define VOLT_ADC_REF      3300U /* 3.3V regulator as reference, fixme, this should be a calibrated variable */

#define HW_V2_VOLT_DIV_R1 4700U  /* Bottom resistor value of the voltage divider, 4.7kΩ */
#define HW_V2_VOLT_DIV_R2 10000U /* Top resistor value of the voltage divider, 10kΩ */

/* Voltage [mV] per unit of ADC value for hardware version 2 and newer */
#define HW_V2_UNIT_MV \
	(((VOLT_ADC_REF * (HW_V2_VOLT_DIV_R1 + HW_V2_VOLT_DIV_R2)) << VOLT_FP_BITS) / (VOLT_ADC_RES * HW_V2_VOLT_DIV_R2))

/* Hardware version 0 and 1 don't have a voltage divider so the calculation is simpler */
#define HW_V0_UNIT_MV ((VOLT_ADC_REF << VOLT_FP_BITS) / VOLT_ADC_RES) /* Voltage [mV] per unit of ADC value */

/*
 * Ensure the fixed point calculation does not overflow by checking a maximum theoretical values, this is a compile time check
 * This is safe up to 16V at 18 fixed point bits, we're not going to be debugging anything at that voltage lol
 */
#define VOLT_THEORETICAL_MAX 5500U /* Maximum theoretical voltage [mV] */
#define UNIT_MV_MAX          ((VOLT_THEORETICAL_MAX << VOLT_FP_BITS) / VOLT_ADC_RES)
#if (UNIT_MV_MAX * (VOLT_ADC_RES - 1U)) > UINT32_MAX
#error "Voltage calculation overflows 32-bit integer"
#endif

uint32_t platform_target_voltage_sense(void)
{
	/*
	 * Returns the voltage in tenths of a volt (so 33 means 3.3V),
	 * except for hardware version 1.
	 * This function is only needed for implementations that allow the
	 * target to be powered from the debug probe
	 */

	/* hardware version 0 and 1 have tpwr connected directly to ADC_IN8 withouth a voltage divider */
	const uint32_t tpwr_unit_mv_fp = hwversion <= 1U ? HW_V0_UNIT_MV : HW_V2_UNIT_MV;

	uint8_t channel = 8;
	adc_set_regular_sequence(ADC1, 1, &channel);
	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1))
		continue;

	/* Convert the ADC value to a voltage in mV, the result will be in fixed point format, shimmy the result a bit */
	const uint32_t tpwr_mv_fp =
		((uint32_t)adc_read_regular(ADC1) * tpwr_unit_mv_fp) + (tpwr_unit_mv_fp / 2U); /* ADC value: 0-4095 */

	/* Clear EOC bit. The GD32F103 does not automatically reset it on ADC read. */
	ADC_SR(ADC1) &= ~ADC_SR_EOC;

	/* Convert to mV discarding the fractional part and rounding to the nearest integer */
	return VOLT_FP_TO_UINT32(tpwr_mv_fp);
}

const char *platform_target_voltage(void)
{
	const uint32_t milivolts = platform_target_voltage_sense();

	static char ret[8U]; /* Enough for "12.345V" */
	snprintf(ret, sizeof(ret), "%lu.%03luV", milivolts / 1000U, milivolts % 1000U);

	// This is cute but breaks compatibility with remote protocol
	// if(tpwr_volt > 0)
	// {
	// 	if (tpwr_mv > 0)
	// 		snprintf(ret, sizeof(ret), "%lu.%03luV", tpwr_volt, tpwr_mv);
	// 	else
	// 		snprintf(ret, sizeof(ret), "%luV", tpwr_volt);
	// }
	// else
	// 	snprintf(ret, sizeof(ret), "%lumV", tpwr_mv);
	return ret;
}

void platform_request_boot(void)
{
	/* Disconnect USB cable */
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);
	gpio_set_mode(USB_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, USB_DP_PIN | USB_DM_PIN);
	gpio_clear(USB_PORT, USB_DP_PIN | USB_DM_PIN);
	/* Make sure we drive the USB reset condition for at least 10ms */
	while (!(STK_CSR & STK_CSR_COUNTFLAG))
		continue;
	for (size_t count = 0U; count < 10U * SYSTICKMS; ++count) {
		while (!(STK_CSR & STK_CSR_COUNTFLAG))
			continue;
	}

	/* Drive boot request pin */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOB, GPIO12);
}

void platform_target_clk_output_enable(bool enable)
{
	if (hwversion >= 6) {
		/* If we're switching to tristate mode, first convert the processor pin to an input */
		if (!enable)
			gpio_set_mode(TCK_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, TCK_PIN);
		/* Reconfigure the logic levelt translator */
		gpio_set_val(TCK_DIR_PORT, TCK_DIR_PIN, enable);
		/* If we're switching back out of tristate mode, we're now safe to make the processor pin an output again */
		if (enable)
			gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);
	}
}

bool platform_spi_init(const spi_bus_e bus)
{
	if (bus == SPI_BUS_EXTERNAL) {
		rcc_periph_clock_enable(RCC_SPI1);
		rcc_periph_reset_pulse(RST_SPI1);
		platform_target_clk_output_enable(true);
		gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TCK_PIN);
		gpio_set_mode(TDI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TDI_PIN);
		gpio_set(TMS_DIR_PORT, TMS_DIR_PIN);
	} else {
		rcc_periph_clock_enable(RCC_SPI2);
		rcc_periph_reset_pulse(RST_SPI2);
	}

	const uint32_t controller = bus == SPI_BUS_EXTERNAL ? EXT_SPI : AUX_SPI;
	spi_init_master(controller, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_enable(controller);
	return true;
}

bool platform_spi_deinit(spi_bus_e bus)
{
	spi_disable(bus == SPI_BUS_EXTERNAL ? EXT_SPI : AUX_SPI);

	if (bus == SPI_BUS_EXTERNAL) {
		rcc_periph_clock_disable(RCC_SPI1);
		gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);
		gpio_set_mode(TDI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TDI_PIN);
		platform_target_clk_output_enable(false);
	} else
		rcc_periph_clock_disable(RCC_SPI2);
	return true;
}

bool platform_spi_chip_select(const uint8_t device_select)
{
	const uint8_t device = device_select & 0x7fU;
	const bool select = !(device_select & 0x80U);
	uint32_t port = AUX_PORT;
	uint16_t pin;
	switch (device) {
	case SPI_DEVICE_INT_FLASH:
		pin = AUX_FCS;
		break;
	case SPI_DEVICE_EXT_FLASH:
		port = EXT_SPI_CS_PORT;
		pin = EXT_SPI_CS;
		break;
	case SPI_DEVICE_SDCARD:
		pin = AUX_SDCS;
		break;
	case SPI_DEVICE_DISPLAY:
		pin = AUX_DCS;
		break;
	default:
		return false;
	}
	gpio_set_val(port, pin, select);
	return true;
}

uint8_t platform_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
	return spi_xfer(bus == SPI_BUS_EXTERNAL ? EXT_SPI : AUX_SPI, value);
}

void exti15_10_isr(void)
{
	uint32_t usb_vbus_port;
	uint16_t usb_vbus_pin;

	if (hwversion < 5) {
		usb_vbus_port = USB_VBUS_PORT;
		usb_vbus_pin = USB_VBUS_PIN;
	} else {
		usb_vbus_port = USB_VBUS5_PORT;
		usb_vbus_pin = USB_VBUS5_PIN;
	}

	if (gpio_get(usb_vbus_port, usb_vbus_pin))
		/* Drive pull-up high if VBUS connected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	else
		/* Allow pull-up to float if VBUS disconnected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);

	exti_reset_request(usb_vbus_pin);
}

static void setup_vbus_irq(void)
{
	uint32_t usb_vbus_port;
	uint16_t usb_vbus_pin;

	if (hwversion < 5) {
		usb_vbus_port = USB_VBUS_PORT;
		usb_vbus_pin = USB_VBUS_PIN;
	} else {
		usb_vbus_port = USB_VBUS5_PORT;
		usb_vbus_pin = USB_VBUS5_PIN;
	}

	nvic_set_priority(USB_VBUS_IRQ, IRQ_PRI_USB_VBUS);
	nvic_enable_irq(USB_VBUS_IRQ);

	gpio_set(usb_vbus_port, usb_vbus_pin);
	gpio_set(USB_PU_PORT, USB_PU_PIN);

	gpio_set_mode(usb_vbus_port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, usb_vbus_pin);

	/* Configure EXTI for USB VBUS monitor */
	exti_select_source(usb_vbus_pin, usb_vbus_port);
	exti_set_trigger(usb_vbus_pin, EXTI_TRIGGER_BOTH);
	exti_enable_request(usb_vbus_pin);

	exti15_10_isr();
}

void dma1_channel5_isr(void)
{
	if (hwversion < 6)
		usart1_rx_dma_isr();
#if SWO_ENCODING != 1
	else
		swo_dma_isr();
#endif
}
