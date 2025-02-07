/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2025 1BitSquared <info@1bitsquared.com>
 * Written by Rafael Silva <perigoso@riseup.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Provides an abstract 'debug interface object',
 * the 'methods' of which must be implemented by a interface driver.
 */

#ifndef INCLUDE_INTERFACE_H
#define INCLUDE_INTERFACE_H

#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*iface_init_func)(void *driver);
typedef void (*iface_deinit_func)(void *driver);
typedef bool (*iface_set_frequency_func)(void *driver, uint32_t freq);
typedef uint32_t (*iface_get_frequency_func)(void *driver);
typedef bool (*iface_scan_func)(void *driver, uint32_t id);

typedef struct interface {
	const char *name;                       /* Interface name */
	iface_init_func init;                   /* Initialize the interface */
	iface_deinit_func deinit;               /* Deinitialize the interface */
	iface_set_frequency_func set_frequency; /* Set the frequency of the interface */
	iface_get_frequency_func get_frequency; /* Set the frequency of the interface */
	iface_scan_func scan;                   /* Scan for targets */
	_Alignas(max_align_t) uint8_t driver[]; /* Interface driver data */
} interface_s;

interface_s *interface_register_driver(const char *name, size_t driver_storage);
#define interface_register_driver_type(name, type) interface_register_driver(name, sizeof(type))

bool interface_init(interface_s *iface);
void interface_deinit(interface_s *iface);

bool interface_scan(interface_s *iface, uint32_t id);

bool interface_set_frequency(interface_s *iface, uint32_t frequency);
uint32_t interface_get_frequency(interface_s *iface);

interface_s *interface_iter_begin(void);
interface_s *interface_iter_next(interface_s *iface);

interface_s *interface_get(const char *name);
void *interface_get_driver(const char *name);

#endif /* INCLUDE_INTERFACE_H */
