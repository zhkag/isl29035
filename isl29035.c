/***********************************************************************************************************************
 * File Name    : i2c_sensor.c
 * Description  : Contains data structures and functions used in i2c_sensor.c.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2019 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/

#define DBG_TAG "sensor.isl29035"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <rtthread.h>
#include <rtdevice.h>

#include <math.h>
#include "hal_data.h"
#include "isl29035.h"

/*
 * Global Variables
 */

uint8_t set_regs_data[2];
struct isl29035_dev isl_dev;

static char *i2c_bus_name = NULL;
static struct rt_i2c_bus_device *i2c_bus = NULL;


/** Performs a write operation on an I2C Master device.
 * @param[in] p_src     Pointer to the location to get write data from.
 * @param[in] bytes     Number of bytes to write.
 */
fsp_err_t hal_i2c_write(uint8_t *const p_src,
                        uint32_t const   bytes)
{
    struct rt_i2c_msg msgs[1];

    msgs[0].addr = (rt_uint16_t)ISL29035_I2C_ADDR;
    msgs[0].buf = p_src;
    msgs[0].len = bytes;
    msgs[0].flags = RT_I2C_WR ;

    if (rt_i2c_transfer(i2c_bus, msgs, 1) == 1)
    {
        return FSP_SUCCESS;
    }
    else
    {
        LOG_E("i2c_write ERROR_I2C");
        return -1;
    }
}

/** Performs a read operation on an I2C Master device.
 * @param[in] p_dest    Pointer to the location to store read data.
 * @param[in] bytes     Number of bytes to read.
 */
fsp_err_t hal_i2c_read(uint8_t *const p_dest,
                       uint32_t const  bytes)
{
    struct rt_i2c_msg msgs[1];
    msgs[0].addr = (rt_uint16_t)ISL29035_I2C_ADDR;
    msgs[0].buf = p_dest;
    msgs[0].len = bytes;
    msgs[0].flags = RT_I2C_RD;

    if (rt_i2c_transfer(i2c_bus, msgs, 1) == 1)
    {
        return FSP_SUCCESS;
    }
    else
    {
        LOG_E("i2c_read ERROR_I2C");
        return -1;
    }
}

fsp_err_t hal_i2c_open(char *bus_name)
{
    i2c_bus_name = bus_name;

    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(i2c_bus_name);
    if (i2c_bus == RT_NULL)
    {
        LOG_E("Device not found!");
        return FSP_ERR_ASSERTION;
    }
    return FSP_SUCCESS;
}

fsp_err_t hal_i2c_close(void)
{

    return FSP_SUCCESS;
}


//ISL29035
fsp_err_t isl29035_get_regs(uint8_t reg_addr,
                            uint8_t *data,
                            uint16_t len,
                            const struct isl29035_dev *dev)
{
    fsp_err_t err         = FSP_SUCCESS;

    /* Get register address to sensor */
    err = hal_i2c_write(&reg_addr, 1);
    /* handle error */
    if (FSP_SUCCESS != err)
    {
        LOG_E("** hal_i2c_write API failed ** \r\n");
    }
    else
    {
        /* Read only when I2C write and its callback event is successful */
        err = hal_i2c_read(data, len);
        /* handle error */
        if (FSP_SUCCESS != err)
        {
            LOG_E("** hal_i2c_read API failed ** \r\n");
            //  Do nothing, the error is returned in the end
        }
    }
    return err;
}

/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 */
fsp_err_t isl29035_set_regs(uint8_t reg_addr,
                            uint8_t *data,
                            uint16_t len,
                            const struct isl29035_dev *dev)
{
    fsp_err_t err         = FSP_SUCCESS;

    set_regs_data[0] = reg_addr;
    set_regs_data[1] = *data;

    err = hal_i2c_write(set_regs_data, len);
    return err;
}

fsp_err_t isl29035_set_mode(struct isl29035_dev *dev)
{
    fsp_err_t err         = FSP_SUCCESS;
    uint8_t val;

    err = isl29035_get_regs(ISL29035_COMMAND_I_REG, &val, 1, dev);
    if (FSP_SUCCESS == err)
    {
        val = (uint8_t)((val & ISL29035_OPMODE_MASK) | dev->adc_mode);

        err = isl29035_set_regs(ISL29035_COMMAND_I_REG, &val, 2, dev);
    }
    return err;
}

fsp_err_t isl29035_set_range(struct isl29035_dev *dev)
{
    fsp_err_t err         = FSP_SUCCESS;
    uint8_t val;

    err = isl29035_get_regs(ISL29035_COMMAND_II_REG, &val, 1, dev);
    if (FSP_SUCCESS == err)
    {
        val = (uint8_t)((val & ISL29035_LUX_RANGE_MASK) | dev->adc_range);

        err = isl29035_set_regs(ISL29035_COMMAND_II_REG, &val, 2, dev);
    }
    return err;
}

fsp_err_t isl29035_set_res(struct isl29035_dev *dev)
{
    fsp_err_t err         = FSP_SUCCESS;
    uint8_t val;

    err = isl29035_get_regs(ISL29035_COMMAND_II_REG, &val, 1, dev);
    if (FSP_SUCCESS == err)
    {
        val = (uint8_t)((val & ISL29035_ADC_RES_MASK) | dev->adc_res);

        err = isl29035_set_regs(ISL29035_COMMAND_II_REG, &val, 2, dev);
    }
    return err;
}

fsp_err_t isl29035_init(struct isl29035_dev *dev, char *bus_name)
{
    uint8_t device_isl29035_id = 0x00;
    fsp_err_t err     = FSP_SUCCESS;
    uint8_t val;

    /* opening IIC master module */
    err = hal_i2c_open(bus_name);
    /* handle error */
    if (FSP_SUCCESS != err)
    {
        LOG_E("** hal_i2c_open API failed ** \r\n");
        return err;
    }

    /* Get device ID as a start of communication */
    err = isl29035_get_regs(ISL29035_REG_DEVICE_ID, &device_isl29035_id, 1, dev);

    if ((FSP_SUCCESS == err) && (ISL29035_CHIP_ID == (device_isl29035_id & ISL29035_DEVICE_ID_MASK)))
    {
        val = (uint8_t)~ISL29035_BOUT_MASK;
        err = isl29035_set_regs(ISL29035_REG_DEVICE_ID, &val, 2, dev);

        if (FSP_SUCCESS != err)
        {
            LOG_E("** isl29035_set_regs API failed ** \r\n");
        }
    }
    else
    {
        /* Failed to get Device ID */
        LOG_E("Error in reading Device ID ** \r\n");
    }
    return err;
}

fsp_err_t isl29035_configure(struct isl29035_dev *dev)
{
    fsp_err_t err         = FSP_SUCCESS;
    uint8_t val;

    /* set command registers to set default attributes */
    val = 0;
    err = isl29035_set_regs(ISL29035_COMMAND_I_REG, &val, 2, dev);

    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    if (err != FSP_SUCCESS)
        return err;

    err = isl29035_set_regs(ISL29035_COMMAND_II_REG, &val, 2, dev);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    if (err != FSP_SUCCESS)
        return err;

    /* Set operation mode */
    dev->adc_mode = ISL29035_ACTIVE_OPMODE_BITS;
    err = isl29035_set_mode(dev);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    if (err != FSP_SUCCESS)
        return err;

    /* Set the lux range */
    dev->adc_range = ISL29035_LUX_RANGE_BITS;
    err = isl29035_set_range(dev);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    if (err != FSP_SUCCESS)
        return err;

    /* Set ADC resolution */
    dev->adc_res = ISL29035_ADC_RES_BITS;
    err = isl29035_set_res(dev);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    if (err != FSP_SUCCESS)
        return err;

    return err;
}

static float compute_scaling(struct isl29035_dev *dev)
{
    fsp_err_t err         = FSP_SUCCESS;
    uint8_t val, reg_val;
    float scale;
    uint8_t adc_res = 0, lux;
    uint16_t lux_range = 0;

    /*
     * Equation is lux = (Range/2 ^ n) * raw_data
     */

    err = isl29035_get_regs(ISL29035_COMMAND_II_REG, &reg_val, 1, dev);
    if (err != FSP_SUCCESS)
        return err;

    //get the ADC range value
    val = (uint8_t)((reg_val & ISL29035_ADC_RES_MASK) >> 2);
    switch (val)
    {
    case 0:
        adc_res = 16;
        break;
    case 1:
        adc_res = 12;
        break;
    case 2:
        adc_res = 8;
        break;
    case 3:
        adc_res = 4;
        break;
    default:
        break;
    }

    //get the ADC resolution value
    lux = (uint8_t)(reg_val & ISL29035_LUX_RANGE_MASK);
    switch (lux)
    {
    case ISL29035_LUX_RANGE_1000:
        lux_range = 1000;
        break;
    case ISL29035_LUX_RANGE_4000:
        lux_range = 4000;
        break;
    case ISL29035_LUX_RANGE_16000:
        lux_range = 16000;
        break;
    case ISL29035_LUX_RANGE_64000:
        lux_range = 64000;
        break;
    default:
        break;
    }

    scale = (float)(lux_range / (pow(2, adc_res)));
    return scale;
}

fsp_err_t isl29035_read_als_data(struct isl29035_dev *dev, double *als_val)
{
    fsp_err_t err         = FSP_SUCCESS;
    uint8_t als_data[2];
    uint16_t als_raw;
    float scale = 0.0;

    err = isl29035_get_regs(ISL29035_DATA_LSB_REG, &als_data[0], 2, dev);
    if (err != FSP_SUCCESS)
        return err;

    als_raw = (uint16_t)((als_data[1] << 8) | als_data[0]);

    //compute scaling factor
    scale = compute_scaling(dev);

    *als_val = (als_raw * scale);

    return err;
}

/*******************************************************************************************************************//**
 *  @brief       DeInitialize isl29035
 *  @param[IN]   None
 *  @retval      None
 **********************************************************************************************************************/
void isl29035_deinit(void)
{
    fsp_err_t err     = FSP_SUCCESS;

    /* close open modules */
    err =  hal_i2c_close();

    if (FSP_SUCCESS != err)
    {
        LOG_E("** hal_i2c_close API failed ** \r\n");
    }
}
