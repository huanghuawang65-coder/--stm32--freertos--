#include "drv_sensor.h"
#include "bsp_adc.h"
#include "bsp_delay.h"
#include "bsp_i2c.h"
#include "board_pins.h"

/* =========================================================
 * 通用传感器接口
 * ========================================================= */

int Sensor_Init(SensorDevice_t *dev)
{
    if (dev == 0 || dev->ops == 0 || dev->ops->init == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    return dev->ops->init(dev);
}

int Sensor_Read(SensorDevice_t *dev, SensorData_t *data)
{
    if (dev == 0 || data == 0 || dev->ops == 0 || dev->ops->read == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    if (dev->state != SENSOR_STATE_READY)
    {
        return SENSOR_ERR_STATE;
    }

    return dev->ops->read(dev, data);
}

int Sensor_Deinit(SensorDevice_t *dev)
{
    if (dev == 0 || dev->ops == 0 || dev->ops->deinit == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    return dev->ops->deinit(dev);
}

/* =========================================================
 * MQ2 气体传感器
 * ========================================================= */

typedef struct
{
    HAL_AdcInput_t adc_channel;
    uint8_t samples;
} GasSensorPriv_t;

static GasSensorPriv_t g_gas_priv = {
    .adc_channel = HAL_ADC_IN_GAS_SENSOR,
    .samples = GAS_ADC_SAMPLES
};

static int Gas_Init(SensorDevice_t *dev)
{
    GasSensorPriv_t *priv;

    if (dev == 0 || dev->private_data == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    priv = (GasSensorPriv_t *)dev->private_data;

    /*
     * MQ2 通过 ADC 采样。
     * 当前 PA1 / ADC_Channel_1 保持不变，V2 原理图没有改 MQ_ADC。
     */
    HAL_ADC_GlobalInit();
    HAL_ADC_InputInit(priv->adc_channel);

    dev->state = SENSOR_STATE_READY;
    return SENSOR_OK;
}

static int Gas_Read(SensorDevice_t *dev, SensorData_t *data)
{
    uint16_t raw;
    GasSensorPriv_t *priv;

    if (dev == 0 || data == 0 || dev->private_data == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    priv = (GasSensorPriv_t *)dev->private_data;
    raw = HAL_ADC_ReadAverage(priv->adc_channel, priv->samples);

    if (raw > 4095u)
    {
        raw = 4095u;
    }

    data->gas_percent = (uint8_t)(((uint32_t)raw * 100u) / 4095u);

    return SENSOR_OK;
}

static int Gas_Deinit(SensorDevice_t *dev)
{
    if (dev == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    dev->state = SENSOR_STATE_UNINIT;
    return SENSOR_OK;
}

static const SensorOps_t g_gas_ops = {
    .init = Gas_Init,
    .read = Gas_Read,
    .deinit = Gas_Deinit
};

SensorDevice_t g_gas_sensor = {
    .name = "mq_sensor",
    .type = SENSOR_TYPE_GAS_SENSOR,
    .state = SENSOR_STATE_UNINIT,
    .private_data = &g_gas_priv,
    .ops = &g_gas_ops
};

/* =========================================================
 * SHT30 温湿度传感器
 * ========================================================= */

#define SHT30_CMD_MEASURE_HIGH_REPEAT_0      0x24u
#define SHT30_CMD_MEASURE_HIGH_REPEAT_1      0x00u
#define SHT30_MEASURE_WAIT_MS                20u

typedef struct
{
    uint8_t address;
    uint8_t last_temp;
    uint8_t last_humi;
} SHT30Priv_t;

static SHT30Priv_t g_sht30_priv = {
    .address = SHT30_I2C_ADDRESS,
    .last_temp = 0,
    .last_humi = 0
};

static uint8_t SHT30_CalcCrc(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t bit;
    uint8_t crc = 0xFFu;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80u)
            {
                crc = (uint8_t)((crc << 1) ^ 0x31u);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static uint8_t SHT30_ClampToU8(int32_t value, int32_t min, int32_t max)
{
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }

    return (uint8_t)value;
}

static int SHT30_ReadRaw(SHT30Priv_t *priv, uint16_t *raw_temp, uint16_t *raw_humi)
{
    uint8_t cmd[2];
    uint8_t rx[6];

    if (priv == 0 || raw_temp == 0 || raw_humi == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    /*
     * 0x2400：High repeatability，clock stretching disabled。
     * 软件 I2C 不处理时钟拉伸，所以发送命令后固定等待测量完成。
     */
    cmd[0] = SHT30_CMD_MEASURE_HIGH_REPEAT_0;
    cmd[1] = SHT30_CMD_MEASURE_HIGH_REPEAT_1;

    if (!BSP_I2C_SHT30_Write(priv->address, cmd, sizeof(cmd)))
    {
        return SENSOR_ERR_IO;
    }

    delay_ms(SHT30_MEASURE_WAIT_MS);

    if (!BSP_I2C_SHT30_Read(priv->address, rx, sizeof(rx)))
    {
        return SENSOR_ERR_IO;
    }

    if (SHT30_CalcCrc(&rx[0], 2) != rx[2] ||
        SHT30_CalcCrc(&rx[3], 2) != rx[5])
    {
        return SENSOR_ERR_CHECKSUM;
    }

    *raw_temp = ((uint16_t)rx[0] << 8) | rx[1];
    *raw_humi = ((uint16_t)rx[3] << 8) | rx[4];

    return SENSOR_OK;
}

static int SHT30_Init_Impl(SensorDevice_t *dev)
{
    if (dev == 0 || dev->private_data == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    BSP_I2C_SHT30_Init();

    dev->state = SENSOR_STATE_READY;
    return SENSOR_OK;
}

static int SHT30_Read_Impl(SensorDevice_t *dev, SensorData_t *data)
{
    int ret;
    uint16_t raw_temp;
    uint16_t raw_humi;
    int32_t temp_c;
    uint32_t humi_percent;
    SHT30Priv_t *priv;

    if (dev == 0 || data == 0 || dev->private_data == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    priv = (SHT30Priv_t *)dev->private_data;

    ret = SHT30_ReadRaw(priv, &raw_temp, &raw_humi);
    if (ret != SENSOR_OK)
    {
        dev->state = SENSOR_STATE_ERROR;
        return ret;
    }

    /*
     * SHT30 数据手册换算公式：
     * T = -45 + 175 * rawT / 65535
     * RH = 100 * rawRH / 65535
     *
     * 当前系统只需要整数温湿度，用整数运算避免引入浮点库。
     */
    temp_c = -45 + (int32_t)(((uint32_t)175u * raw_temp) / 65535u);
    humi_percent = ((uint32_t)100u * raw_humi) / 65535u;

    data->temperature = SHT30_ClampToU8(temp_c, 0, 100);
    data->humidity = SHT30_ClampToU8((int32_t)humi_percent, 0, 100);

    priv->last_temp = data->temperature;
    priv->last_humi = data->humidity;
    dev->state = SENSOR_STATE_READY;

    return SENSOR_OK;
}

static int SHT30_Deinit_Impl(SensorDevice_t *dev)
{
    if (dev == 0)
    {
        return SENSOR_ERR_PARAM;
    }

    dev->state = SENSOR_STATE_UNINIT;
    return SENSOR_OK;
}

static const SensorOps_t g_sht30_ops = {
    .init = SHT30_Init_Impl,
    .read = SHT30_Read_Impl,
    .deinit = SHT30_Deinit_Impl
};

SensorDevice_t g_sht30_sensor = {
    .name = "sht30_sensor",
    .type = SENSOR_TYPE_SHT30,
    .state = SENSOR_STATE_UNINIT,
    .private_data = &g_sht30_priv,
    .ops = &g_sht30_ops
};
