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

#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "interface.h"
#include "llist.h"

static void interface_dtor(void *interface_storage)
{
	interface_s *const iface = (interface_s *)interface_storage;
	interface_deinit(iface);
}

static llist_s interface_list = llist_init_dtor(interface_dtor);
interface_s *active_interface = NULL;

interface_s *interface_register_driver(const char *const name, const size_t driver_storage)
{
	/* Ensure the interface name is unique */
	if (interface_get(name) != NULL) {
		return NULL;
	}

	/* Allocate memory for the interface and driver */
	interface_s *const iface = llist_append_new(&interface_list, sizeof(interface_s) + driver_storage);
	if (iface == NULL) {
		return NULL;
	}
	iface->name = name;
	return iface;
}

bool interface_init(interface_s *const iface)
{
	/* Required method, no check if exists */
	if (active_interface != iface) {
		if (active_interface != NULL)
			interface_deinit(active_interface);
		active_interface = iface;
	}
	return iface->init(iface->driver);
}

void interface_deinit(interface_s *const iface)
{
	if (iface->deinit != NULL)
		iface->deinit(iface->driver);
	active_interface = NULL;
}

bool interface_scan(interface_s *const iface, const uint32_t id)
{
	/* Required method, no check if exists */
	return iface->scan(iface->driver, id);
}

bool interface_set_frequency(interface_s *const iface, const uint32_t frequency)
{
	if (iface->set_frequency == NULL)
		return false;
	return iface->set_frequency(iface->driver, frequency);
}

uint32_t interface_get_frequency(interface_s *const iface)
{
	if (iface->get_frequency == NULL)
		return 0U;
	return iface->get_frequency(iface->driver);
}

interface_s *interface_iter_begin(void)
{
	return llist_begin(&interface_list);
}

interface_s *interface_iter_next(interface_s *const iface)
{
	return llist_next(iface);
}

interface_s *interface_get(const char *const name)
{
	llist_for_each(interface_s, iface, &interface_list)
	{
		if (strcmp(iface->name, name) == 0)
			return iface;
	}
	return NULL;
}

void *interface_get_driver(const char *name)
{
	interface_s *const iface = interface_get(name);
	if (iface == NULL) {
		return NULL;
	}
	return iface->driver;
}
