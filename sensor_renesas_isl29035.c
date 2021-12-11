/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-10     Sherman      the first version
 */

#define DBG_TAG "sensor.isl29035"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "hal_data.h"
#include "sensor_renesas_isl29035.h"
#include "isl29035.h"

#define SENSOR_LIGHT_RANGE_MAX (ISL29035_LUX_RANGE)
#define SENSOR_LIGHT_RANGE_MIN (0)

struct isl29035_device
{
    struct rt_i2c_bus_device *i2c;
    rt_uint8_t addr;
    struct isl29035_dev dev;
};
static struct isl29035_device isl29035_device;

static rt_err_t _isl29035_init(struct rt_sensor_intf *intf)
{
    rt_int8_t ret;
    isl29035_device.i2c = rt_i2c_bus_device_find(intf->dev_name);
    if (isl29035_device.i2c == RT_NULL)
    {
        return -RT_ERROR;
    }

    isl29035_device.dev.id = ISL29035_I2C_ADDR;
    isl29035_device.dev.interface = ISL29035_I2C_INTF;

    ret = isl29035_init(&isl29035_device.dev, intf->dev_name);
    if (ret)
    {
        LOG_E("Error %d during initialize hardware, exiting program!\n", ret);
        goto __exit_err;
    }

    ret = isl29035_configure(&isl29035_device.dev);

    if (FSP_SUCCESS != ret)
    {
        /* Sensor init failed, so cleanup the sensor specific initialization */
        LOG_E("** SENSOR ISL29035 CONFIGURATION FAILED ** \r\n");
        isl29035_deinit();
        goto __exit_err;
    }

    return RT_EOK;
__exit_err:
    return -RT_ERROR;
}

static void isl29035_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    RT_ASSERT(sensor != RT_NULL);
    RT_ASSERT(data != RT_NULL);
    rt_uint8_t ret;
    double als_value; // Ambient Light sensor

    /* Read sensor data */
    ret = isl29035_read_als_data(&isl29035_device.dev, &als_value);
    if (ret)
    {
        LOG_E("Error %d! SENSOR READ DATA FAILED\n", ret);
        goto __exit_err;
    }

    if (sensor->info.type == RT_SENSOR_CLASS_LIGHT)
    {
        data->data.light = als_value;
    }

__exit_err:
    return;
}

static rt_size_t isl29035_fetch_data(struct rt_sensor_device *sensor,
                                     void *buf,
                                     rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        isl29035_polling_get_data(sensor, buf);
        return 1;
    }
    else
        return 0;
}

static rt_err_t isl29035_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    isl29035_fetch_data,
    isl29035_control
};

int rt_hw_isl29035_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_light = RT_NULL;

    /* isl29035 LIGHT sensor register */
    sensor_light = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_light == RT_NULL)
        return -1;

    sensor_light->info.type       = RT_SENSOR_CLASS_LIGHT;
    sensor_light->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
    sensor_light->info.model      = "isl29035";
    sensor_light->info.unit       = RT_SENSOR_UNIT_LUX;
    sensor_light->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor_light->info.range_max  = SENSOR_LIGHT_RANGE_MAX;
    sensor_light->info.range_min  = SENSOR_LIGHT_RANGE_MIN;
    sensor_light->info.period_min = 3000;
    sensor_light->ops = &sensor_ops;
    rt_memcpy(&sensor_light->config, cfg, sizeof(struct rt_sensor_config));

    result = rt_hw_sensor_register(sensor_light,
                                   name,
                                   RT_DEVICE_FLAG_RDONLY,
                                   RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        goto __exit_err;
    }

    if (RT_EOK != _isl29035_init(&cfg->intf))
        goto __exit_err;

    return RT_EOK;
__exit_err:
    rt_device_unregister(&sensor_light->parent);

    if (sensor_light)
        rt_free(sensor_light);
    return -RT_ERROR;
}
