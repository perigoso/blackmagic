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

/* This file implements a simple linked list that can be used with any element type */

#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include "llist.h"

static inline llist_element_s **llist_find_indirect(llist_s *const list, void *const element_data)
{
	llist_element_s **indirect = &list->head;
	while (*indirect != NULL && llist_dereference_element(*indirect) != element_data) {
		indirect = &(*indirect)->next;
	}
	return indirect;
}

static llist_element_s *llist_element_alloc(llist_s *const list, const size_t element_size)
{
	if (element_size == 0)
		return NULL;
	llist_element_s *const element = calloc(1U, sizeof(llist_element_s) + element_size);
	if (element != NULL) {
		if (list->element_ctor)
			list->element_ctor(llist_dereference_element(element));
	}
	return element;
}

static void llist_element_free(llist_s *const list, llist_element_s *const element)
{
	if (list->element_dtor)
		list->element_dtor(llist_dereference_element(element));
	free(element);
}

static inline void llist_insert_before(llist_element_s **const indirect, llist_element_s *const element)
{
	element->next = *indirect;
	*indirect = element;
}

void *llist_append_new(llist_s *const list, const size_t element_size)
{
	llist_element_s *const element = llist_element_alloc(list, element_size);
	if (element != NULL)
		llist_insert_before(llist_find_indirect(list, NULL), element);
	return llist_dereference_element(element);
}

void *llist_prepend_new(llist_s *const list, const size_t element_size)
{
	llist_element_s *const element = llist_element_alloc(list, element_size);
	if (element != NULL)
		llist_insert_before(&list->head, element);
	return llist_dereference_element(element);
}

void llist_remove(llist_s *const list, void *const element)
{
	llist_element_s **indirect = llist_find_indirect(list, element);
	if (*indirect != NULL) {
		llist_element_s *to_remove = *indirect;

		*indirect = to_remove->next;
		llist_element_free(list, to_remove);
	}
}

void llist_destroy(llist_s *const list)
{
	for (llist_element_s *to_remove = list->head; to_remove != NULL;) {
		llist_element_s *const next = to_remove->next;
		llist_element_free(list, to_remove);
		to_remove = next;
	}
	list->head = NULL;
}
