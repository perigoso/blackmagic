/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2023 1BitSquared <info@1bitsquared.com>
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

#ifndef INCLUDE_LIST_H
#define INCLUDE_LIST_H

#include <stddef.h>
#include <stdbool.h>

typedef struct list_node list_node_t;
typedef struct list list_t;

typedef struct list_node {
	struct list_node *next;
	void *data;
} list_node_t;

typedef void *(list_constructor_t)();
typedef void(list_destructor_t)(void *);

typedef struct list {
	list_constructor_t *constructor;
	list_destructor_t *destructor;
	list_node_t *head;
} list_t;

#define list_node_data(type, node) ((type *)((node)->data))
#define LIST_END                   NULL

/* List access */
list_node_t *list_back(list_t *list);
list_node_t *list_at(list_t *list, size_t index);

void *list_data_at(list_t *list, size_t index);
void *list_data(list_node_t *node);

/* List iterators */
list_node_t *list_begin(list_t *list);
list_node_t *list_next(list_node_t *node);

/* List capacity */
size_t list_size(list_t *list);
bool list_empty(list_t *list);

/* List modifiers */
void list_clear(list_t *list);
void list_erase(list_t *list, list_node_t *node);
list_node_t *list_insert(list_t *list, list_node_t *node, void *data);
list_node_t *list_push_back(list_t *list, void *data);
list_node_t *list_emplace_back(list_t *list);
void list_swap(list_node_t *node, list_node_t *other_node);

#endif /* INCLUDE_LIST_H */
