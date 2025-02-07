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

#ifndef INCLUDE_LLIST_H
#define INCLUDE_LLIST_H

#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (*llist_ctor_fn)(void *);
typedef void (*llist_dtor_fn)(void *);

typedef struct llist_element llist_element_s;

struct llist_element {
	llist_element_s *next;
	_Alignas(max_align_t) char data[];
};

typedef struct llist {
	llist_element_s *head;
	llist_ctor_fn element_ctor;
	llist_dtor_fn element_dtor;
} llist_s;

#define llist_init_ctor_dtor(_ctor, _dtor)                         \
	{                                                              \
		.head = NULL, .element_ctor = _ctor, .element_dtor = _dtor \
	}
#define llist_init_dtor(_dtor) llist_init_ctor_dtor(NULL, _dtor)
#define llist_init_ctor(_ctor) llist_init_ctor_dtor(_ctor, NULL)
#define llist_init()           llist_init_ctor_dtor(NULL, NULL)

static inline void *llist_dereference_element(llist_element_s *const element)
{
	return element->data;
}

static inline llist_element_s *llist_dereference_data(void *const element_data)
{
	return (llist_element_s *)((char *)element_data - offsetof(llist_element_s, data));
}

void *llist_append_new(llist_s *list, size_t element_size);
void *llist_prepend_new(llist_s *list, size_t element_size);
void llist_remove(llist_s *list, void *element);

void llist_destroy(llist_s *list);

static inline void *llist_begin(llist_s *const list)
{
	return llist_dereference_element(list->head);
}

static inline void *llist_next(void *const element)
{
	return llist_dereference_element(llist_dereference_data(element)->next);
}

#define llist_for_each(_type, _element, _list) \
	for (_type *_element = llist_begin(_list); _element != NULL; _element = llist_next(_element))

static inline bool llist_empty(llist_s *list)
{
	return list->head == NULL;
}

static inline size_t llist_size(llist_s *list)
{
	size_t size = 0;
	llist_for_each(void, element, list)
	{
		++size;
	}
	return size;
}

#endif /* INCLUDE_LLIST_H */
