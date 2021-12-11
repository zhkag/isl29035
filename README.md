# ISL29035 光线传感器

## 简介

ISL29035 软件包使用了光线传感器 `ISL29035` 的基本功能。

传感器 `ISL29035` 适用于2.25V至3.63V的电源，I2C电源范围为1.7V至3.63V，适用于-40°C至+85°C的环境温度范围，[ISL29035 详细功能参数介绍](https://www2.renesas.cn/cn/zh/products/sensor-products/light-proximity-sensors/ambient-light-sensors/ambient-light-digital-sensors/isl29035-integrated-digital-light-sensor-interrupt)。

## 支持情况

| 设备 | LIGHT |
| ------- | ---- | 
| **通信接口** |  |
| IIC     |   √  |
| **工作模式** |  |
| 轮询    |  √   |
| 中断    |      |
| FIFO    |      |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：isl29035 设备使用 IIC 进行数据通讯，需要系统 IIC 驱动支持

### 获取软件包

使用 isl29035 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
            ISL29035: Integrated Digital Light Sensor ISL29035 driver library.
                    Version (latest)  --->
```

**Version**：软件包版本选择，默认选择最新版本。

### 使用软件包

isl29035 的初始化函数如下所示：
```c
int rt_hw_isl29035_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息配置接口设备）；
- 注册相应的传感器设备，完成 isl29035 传感器设备的注册；

#### 初始化示例
```c
#include "sensor_renesas_isl29035.h"
#define ISL29035_I2C_BUS "i2c1"
int rt_hw_isl29035_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.dev_name  = ISL29035_I2C_BUS;
    rt_hw_isl29035_init("isl29035", &cfg);
    return RT_EOK;
}
INIT_ENV_EXPORT(rt_hw_isl29035_port);

```
#### 读取数据

- 数据已接入 rt-thread 传感器框架，可以使用 `sensor` 相关命令读取传感器信息
  - 测试命令 `sensor_polling li_isl29` ，验证能否读取 IAQ 数据。


运行效果如下：

```shell
msh >sensor_polling li_isl29
[I/sensor.cmd] num:  0, light:  444 lux, timestamp:40208
[I/sensor.cmd] num:  1, light:  444 lux, timestamp:40208
[I/sensor.cmd] num:  2, light:  444 lux, timestamp:40208
[I/sensor.cmd] num:  3, light:  444 lux, timestamp:40208
[I/sensor.cmd] num:  4, light:  444 lux, timestamp:40208
[I/sensor.cmd] num:  5, light:  444 lux, timestamp:40208
[I/sensor.cmd] num:  6, light:  444 lux, timestamp:40208
[I/sensor.cmd] num:  7, light:  441 lux, timestamp:40208
[I/sensor.cmd] num:  8, light:  412 lux, timestamp:40208
[I/sensor.cmd] num:  9, light:  412 lux, timestamp:40208

```

## 注意事项

- 无

## 联系人信息

维护人:

- 维护：[Sherman](shaopengyu@rt-thread.com)
- 主页：https://github.com/ShermanShao/isl29035
