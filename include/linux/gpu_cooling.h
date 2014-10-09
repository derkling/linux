/*
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GPU_COOLING_H__
#define __GPU_COOLING_H__

#include <linux/thermal.h>

struct thermal_cooling_device *gpu_cooling_register(void);
void gpu_cooling_unregister(struct thermal_cooling_device *cdev);

#endif	/* __GPU_COOLING_H__ */
