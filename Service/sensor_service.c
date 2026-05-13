#include "sensor_service.h"
#include "drv_sensor.h"
#include "message_def.h"
#include "log_service.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "system_state.h"

/**
 * @file sensor_service.c
 * @brief 传感器业务服务。
 *
 * 本服务负责周期读取 MQ2 和 SHT30：
 * 1. MQ2 通过 PA1/ADC_Channel_1 获取气体浓度百分比；
 * 2. SHT30 通过 PC0/PC1 软件 I2C 获取温湿度；
 * 3. 对采样值做基本合法性检查；
 * 4. 只把本轮有效的数据写入 g_system_state，避免某个传感器失败时把显示值刷成 0；
 * 5. 通过日志服务输出当前环境数据，供串口助手观察。
 */

#define SYS_DATA_LOCK_TIMEOUT_MS      50

#define SENSOR_GAS_PERCENT_MAX        100
#define SENSOR_HUMIDITY_MAX           100
#define SENSOR_TEMPERATURE_MAX        80

typedef struct
{
    SensorData_t data;
    uint8_t gas_valid;
    uint8_t sht30_valid;
} SensorSample_t;

static void Sensor_Service_UpdateSystemData(const SensorSample_t *sample)
{
    if (sample == 0 || sys_data_mutex == 0)
    {
        return;
    }

    if (xSemaphoreTake(sys_data_mutex, pdMS_TO_TICKS(SYS_DATA_LOCK_TIMEOUT_MS)) == pdTRUE)
    {
        if (sample->gas_valid)
        {
            g_system_state.sensor.gas_percent = sample->data.gas_percent;
        }

        if (sample->sht30_valid)
        {
            g_system_state.sensor.temperature = sample->data.temperature;
            g_system_state.sensor.humidity = sample->data.humidity;
        }

        xSemaphoreGive(sys_data_mutex);
    }
}

static uint8_t Sensor_Service_IsGasValueValid(const SensorData_t *data)
{
    return (data != 0 && data->gas_percent <= SENSOR_GAS_PERCENT_MAX);
}

static uint8_t Sensor_Service_IsSht30ValueValid(const SensorData_t *data)
{
    /*
     * SHT30 本身会做 CRC 校验，这里只做业务层范围检查。
     * 如果后续项目需要支持低温环境，可以把 temperature 改成有符号类型。
     */
    return (data != 0 &&
            data->humidity <= SENSOR_HUMIDITY_MAX &&
            data->temperature <= SENSOR_TEMPERATURE_MAX);
}

static void Sensor_Service_ReadSample(SensorSample_t *sample)
{
    int ret;

    if (sample == 0)
    {
        return;
    }

    sample->gas_valid = 0;
    sample->sht30_valid = 0;

    ret = Sensor_Read(&g_gas_sensor, &sample->data);
    if (ret == SENSOR_OK && Sensor_Service_IsGasValueValid(&sample->data))
    {
        sample->gas_valid = 1;
    }

    ret = Sensor_Read(&g_sht30_sensor, &sample->data);
    if (ret == SENSOR_OK && Sensor_Service_IsSht30ValueValid(&sample->data))
    {
        sample->sht30_valid = 1;
    }
    else
    {
        /*
         * SHT30 读取失败通常来自 I2C 未应答、CRC 错误或接线问题。
         * 重新初始化软件 I2C 后，下一轮任务会继续尝试读取。
         */
        Sensor_Deinit(&g_sht30_sensor);
        Sensor_Init(&g_sht30_sensor);
    }
}

static void Sensor_Service_PrintSample(const SensorSample_t *sample)
{
    if (sample == 0 || (!sample->gas_valid && !sample->sht30_valid))
    {
        return;
    }

    if (sample->gas_valid && sample->sht30_valid)
    {
        LOG_INFO("MQ2 Gas=%d%%, Temp=%dC, Humi=%d%%",
                 sample->data.gas_percent,
                 sample->data.temperature,
                 sample->data.humidity);
    }
    else if (sample->gas_valid)
    {
        LOG_WARN("MQ2 Gas=%d%%, SHT30 invalid",
                 sample->data.gas_percent);
    }
    else
    {
        LOG_WARN("MQ2 invalid, Temp=%dC, Humi=%d%%",
                 sample->data.temperature,
                 sample->data.humidity);
    }
}

void Sensor_Service_Init(void)
{
    Sensor_Init(&g_gas_sensor);
    Sensor_Init(&g_sht30_sensor);
}

void Sensor_Service_Process(void)
{
    SensorSample_t sample = {0};

    Sensor_Service_ReadSample(&sample);
    Sensor_Service_UpdateSystemData(&sample);
    Sensor_Service_PrintSample(&sample);
}
