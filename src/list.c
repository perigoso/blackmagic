/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
 * All rights reserved.
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

#include "list.h"
#include <stdlib.h>

/**
 * Find an item in the list.
 *
 * Finds the item identified by `target` in the list `l` and returns a
 * the address of the `next` pointer of the item before `target`.
 * This is a linear O(n) search.
 *
 * returns the pointer to the `next` pointer of the item that precedes
 *         `target`, or `head`. Pointer to the last `next` field in the list
 *         if `target` is not in `l`.
 */
static list_node_t **list_find_indirect(list_t *list, list_node_t *target_node)
{
	list_node_t **next_node_pointer = &list->head;
	while (*next_node_pointer != target_node)
		next_node_pointer = &(*next_node_pointer)->next;
	return next_node_pointer;
}

static list_node_t *list_create_node(list_t *list, void *data)
{
	list_node_t *node = calloc(1U, sizeof(list_node_t));
	if (node == NULL)
		return NULL;

	if (data != NULL)
		node->data = data;
	else if (list->constructor != NULL)
		node->data = list->constructor();

	return node;
}

static void list_destroy_node(list_t *list, list_node_t *node)
{
	if (list->destructor != NULL)
		list->destructor(node->data);
	free(node);
}

static list_node_t *list_insert_node(list_t *list, list_node_t *before, list_node_t *node)
{
	list_node_t **node_pointer = list_find_indirect(list, before);
	*node_pointer = node;
	node->next = before;
	return node;
}

list_node_t *list_back(list_t *list)
{
	list_node_t *node = list_begin(list);
	for (; list_next(node) != LIST_END; node = list_next(node))
		continue;
	return node;
}

list_node_t *list_at(list_t *list, size_t index)
{
	list_node_t *node = list_begin(list);
	for (size_t i = 0; i < index && node != LIST_END; ++i)
		node = list_next(node);
	return node;
}

void *list_data_at(list_t *list, size_t index)
{
	return list_data(list_at(list, index));
}

void *list_data(list_node_t *node)
{
	return node != NULL ? node->data : NULL;
}

list_node_t *list_begin(list_t *list)
{
	return list->head;
}

list_node_t *list_next(list_node_t *node)
{
	return node->next;
}

size_t list_size(list_t *list)
{
	size_t size = 0;
	for (list_node_t *node = list_begin(list); node != LIST_END; node = list_next(node), ++size)
		continue;
	return size;
}

bool list_empty(list_t *list)
{
	return list_begin(list) == LIST_END;
}

void list_clear(list_t *list)
{
	for (list_node_t *node = list_begin(list); node != LIST_END;) {
		list_node_t *const next_node = list_next(node);

		list_destroy_node(list, node);
		node = next_node;
	}
}

void list_erase(list_t *list, list_node_t *node)
{
	list_node_t **node_pointer = list_find_indirect(list, node);
	*node_pointer = node->next;

	if (list->destructor != NULL)
		list->destructor(node->data);
	free(node);
}

list_node_t *list_insert(list_t *list, list_node_t *node, void *data)
{
	return list_insert_node(list, node, list_create_node(list, data));
}

list_node_t *list_push_back(list_t *list, void *data)
{
	return list_insert(list, LIST_END, data);
}

list_node_t *list_emplace_back(list_t *list)
{
	return list_push_back(list, NULL);
}

void list_swap(list_node_t *node, list_node_t *other_node)
{
	void *const data_holder = node->data;
	node->data = other_node->data;
	other_node->data = data_holder;
}
