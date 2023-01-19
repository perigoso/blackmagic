/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2023 1BitSquared <info@1bitsquared.com>
 * Written by Rafael Silva <perigoso@riseup.net>
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

/* This file implements Winner Micro targets (W600, W601) */

/*
scan_multidrop: false
DP DPIDR 0x2ba01477 (v1 rev0) designer 0x43b partno 0xba
RESET_SEQ failed
AP   0: IDR=24770011 CFG=00000000 BASE=e00ff003 CSW=23000040 (AHB-AP var1 rev2)
Halt via DHCSR: success 00030003 after 0ms
ROM: Table BASE=0xe00ff000 SYSMEM=0x00000001, Manufacturer 43b Partno 4c3
0 0xe000e000: Generic IP component - Cortex-M3 SCS (System Control Space) (PIDR = 0x00000004000bb000  DEVTYPE = 0x00 ARCHID = 0x0000)
-> cortexm_probe
CPUID 0x412fc231 (M3 var 2 rev 1)
DBGMCU_IDCODE 0x0, DEVID 0x0, REVID 0x0
1 0xe0001000: Generic IP component - Cortex-M3 DWT (Data Watchpoint and Trace) (PIDR = 0x00000004003bb002  DEVTYPE = 0x00 ARCHID = 0x0000)
2 0xe0002000: Generic IP component - Cortex-M3 FBP (Flash Patch and Breakpoint) (PIDR = 0x00000004002bb003  DEVTYPE = 0x00 ARCHID = 0x0000)
3 Entry 0xfff01002 -> Not present
4 Entry 0xfff41002 -> Not present
5 Entry 0xfff42002 -> Not present
ROM: Table END
*/

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

bool winner_probe(target_s *t)
{
	// target_add_ram(t, 0x20000000, 0x5000);
	// ch32f1_add_flash(t, FLASH_BEGIN_ADDRESS_CH32, flashSize * 1024U, 128);
	t->driver = "W600";

	return true;
}
