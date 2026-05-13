#ifndef __DRV_SENSOR_H
#define __DRV_SENSOR_H

#include <stdint.h>

/**
 * @file drv_sensor.h
 * @brief 传感器驱动抽象接口。
 *
 * Driver 层只负责和具体硬件交互，并把采样结果写入 SensorData_t。
 * 数据过滤、系统状态缓存、串口日志和自动控制逻辑都放在 Service 层。
 */

typedef enum
{
    SENSOR_OK = 0,              /* 读取成功。 */
    SENSOR_ERR_PARAM = -1,      /* 参数错误，例如空指针。 */
    SENSOR_ERR_STATE = -2,      /* 设备状态不允许当前操作。 */
    SENSOR_ERR_TIMEOUT = -3,    /* 设备应答超时。 */
    SENSOR_ERR_CHECKSUM = -4,   /* CRC/校验和错误。 */
    SENSOR_ERR_IO = -5          /* 总线读写失败。 */
} SensorStatus_t;

typedef enum
{
    SENSOR_STATE_UNINIT = 0,    /* 设备尚未初始化。 */
    SENSOR_STATE_READY,         /* 设备已经初始化，可以读取。 */
    SENSOR_STATE_ERROR          /* 最近一次读取失败，需要上层重新初始化或下次重试。 */
} SensorState_t;

typedef enum
{
    SENSOR_TYPE_SHT30 = 0,      /* SHT30 温湿度传感器。 */
    SENSOR_TYPE_GAS_SENSOR      /* MQ 系列气体传感器 ADC 输入。 */
} SensorType_t;

/**
 * @brief 系统统一传感器数据结构。
 *
 * MQ2 只更新 gas_percent；
 * SHT30 只更新 temperature 和 humidity；
 * Service 层会根据本轮哪个传感器读取成功，只刷新对应字段。
 */
typedef struct
{
    uint8_t gas_percent;        /* 气体浓度百分比，0~100。 */
    uint8_t temperature;        /* 温度整数值，单位：摄氏度。 */
    uint8_t humidity;           /* 相对湿度整数值，单位：%。 */
} SensorData_t;

struct SensorDevice;

typedef struct
{
    int (*init)(struct SensorDevice *dev);
    int (*read)(struct SensorDevice *dev, SensorData_t *data);
    int (*deinit)(struct SensorDevice *dev);
} SensorOps_t;

typedef struct SensorDevice
{
    const char *name;           /* 设备名称，主要用于调试和日志。 */
    SensorType_t type;          /* 设备类型，区分 MQ2 / SHT30。 */
    SensorState_t state;        /* 设备当前状态，由驱动维护。 */
    void *private_data;         /* 设备私有上下文，业务层不要直接访问。 */
    const SensorOps_t *ops;     /* 设备操作表，实现多态式调用。 */
} SensorDevice_t;

int Sensor_Init(SensorDevice_t *dev);
int Sensor_Read(SensorDevice_t *dev, SensorData_t *data);
int Sensor_Deinit(SensorDevice_t *dev);

extern SensorDevice_t g_gas_sensor;
extern SensorDevice_t g_sht30_sensor;

#endif /* __DRV_SENSOR_H */
