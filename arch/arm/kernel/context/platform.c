/*
 * Copyright (C) 2008-2010 ARM Limited
 *
 * This software is provided 'as-is', without any express or implied
 * warranties including the implied warranties of satisfactory quality,
 * fitness for purpose or non infringement.  In no event will  ARM be
 * liable for any damages arising from the use of this software.
 *
 * Permission is granted to anyone to use, copy and modify this software for
 * any purpose, and to redistribute the software, subject to the following
 * restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/cpumask.h>
#include <asm/memory.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/cputype.h>
#include <asm/lb_lock.h>
#include <asm/appf_platform_api.h>
#include "appf_internals.h"
#include "appf_helpers.h"

#ifndef MAX_CPUS
#define MAX_CPUS 4
#endif

#define APPF_CACHED 0
#define APPF_UNCACHED 1


struct appf_cluster  aem_cluster[1];
struct appf_cpu  *aem_cpu;

void *context_memory;
void *context_memory_uncached;
/*
 * Simple memory allocator function.
 * Returns start address of allocated region
 * Allocates region of size bytes, size will be rounded up to multiple of
 * sizeof(long long)
 * Memory is zero-initialized.
 */
unsigned watermark, watermark_uncached = 0;

static unsigned  get_memory(unsigned size, unsigned type)
{
	unsigned ret;
	void *vmem =  NULL;

	switch (type) {
	case APPF_CACHED:
		ret = watermark;
		watermark += size;
		BUG_ON(watermark >= CONTEXT_SPACE);
		vmem = (context_memory + ret);
		break;
	case APPF_UNCACHED:
		ret = watermark_uncached;
		watermark_uncached += size;
		BUG_ON(watermark_uncached >= CONTEXT_SPACE_UNCACHED);
		vmem = (context_memory_uncached + ret);
		break;
	default:
		printk(KERN_WARNING "Allocating unknown memory type\n");
		break;
	}

	return (unsigned) vmem;
}


/*
 * This function returns the index in the CPU table for the currently
 * executing CPU.
 * Normally, reading the MPIDR is sufficient.
 *
 * This function must be in the entry point section.
 */
int  appf_platform_get_cpu_index(void)
{
	return read_mpidr() & 0xff;
}

/**
 * This function returns the index in the cluster table for the currently executing CPU.
 * Normally, either returning zero or reading the MPIDR is sufficient.
 *
 * This function must be in the entry point section.
 */
int  appf_platform_get_cluster_index(void)
{
	return 0;
}

#include "platform_v7.c"

int  appf_platform_init(void)
{
	return arch->init();
}
