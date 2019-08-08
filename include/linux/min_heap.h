/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_MIN_HEAP_H
#define _LINUX_MIN_HEAP_H

#include <linux/string.h>
#include <linux/bug.h>

/*
 * Data structure used to hold a min-heap, ordered by group_index, of a fixed
 * maximum size.
 */
struct min_heap {
	void *storage;
	int num;
	int max;
};

struct min_heap_callbacks {
	int size;
	int (*cmp)(const void *, const void *);
	void (*swp)(void *, void *);
};

/* Sift the perf_event at pos down the heap. */
static inline void min_heapify(struct min_heap *heap, int pos,
		const struct min_heap_callbacks *func)
{
	void *left, *right, *pivot, *half;

	half = heap->storage + (heap->num / 2) * func->size;
	pivot = heap->storage + pos * func->size;

	while (pivot < half) {
		left = heap->storage + (pos * 2) * func->size;
		right = left + func->size;

		if (func->cmp(pivot, left) > 0) {
			func->swp(pivot, left);
			pivot = left;
			pos = (pos * 2);
		} else if (func->cmp(pivot, right) > 0) {
			func->swp(pivot, right);
			pivot = right;
			pos = (pos * 2) + 1;
		} else {
			break;
		}
	}
}

/* Floyd's approach to heapification that is O(n). */
static inline void
min_heapify_all(struct min_heap *heap, const struct min_heap_callbacks *func)
{
	int i;

	for (i = heap->num / 2; i > 0; i--)
		min_heapify(heap, i, func);
}

/* Remove minimum element from the heap. */
static inline void
min_heap_pop(struct min_heap *heap, const struct min_heap_callbacks *func)
{
	if (WARN_ONCE(heap->num <= 0, "Popping an empty heap"))
		return;

	heap->num--;
	memcpy(heap->storage, heap->storage + (heap->num * func->size), func->size);
	min_heapify(heap, 0, func);
}

static inline void
min_heap_push(struct min_heap *heap, void *data, const struct min_heap_callbacks *func)
{
	void *curr, *parent;
	int pos;

	if (!data)
		return;

	if (WARN_ONCE(heap->num >= heap->max, "Pushing a full heap"))
		return;

	pos = ++heap->num;

	curr = heap->storage + (pos * func->size);
	memcpy(curr, data, func->size);

	for (;;) {
		pos /= 2;
		parent = heap->storage + (pos * func->size);
		if (func->cmp(curr, parent) <= 0)
			break;
		func->swp(curr, parent);
		curr = parent;
	}
}

#endif /* _LINUX_MIN_HEAP_H */
