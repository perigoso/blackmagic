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

/* This file implements the platform specific functions for the ST-Link
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
// #include "usbuart.h"

uint16_t led_idle_run;
uint16_t srst_pin;
static uint32_t rev;
static void adc_init(void);

int platform_hwversion(void)
{
	return rev;
}

void platform_init(void)
{
	rev = 0x5A5A5A5A;
#ifdef ENABLE_DEBUG
	void initialise_monitor_handles(void);
	initialise_monitor_handles();
#endif

	platform_srst_set_val(false);


	platform_timing_init();

	cdcacm_init();
	adc_init();
}

void platform_srst_set_val(bool assert)
{
	if (assert) {
	} else {
	}
}

bool platform_srst_get_val()
{
	return 0;
}

static void adc_init(void)
{

}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.00V";
	return ret;
}
