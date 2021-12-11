/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-16     Sherman      the first version
 */

#ifndef SENSOR_RENESAS_ISL29035_H__
#define SENSOR_RENESAS_ISL29035_H__

#include "sensor.h"

int rt_hw_isl29035_init(const char *name, struct rt_sensor_config *cfg);

#endif
