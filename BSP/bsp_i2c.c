#include "bsp_i2c.h"
#include "board_pins.h"
#include "bsp_delay.h"

#define I2C_ACK                     0u
#define I2C_NACK                    1u

/*
 * 软件 I2C 使用开漏输出。
 * 写 1 表示释放总线，由外部上拉电阻把 SCL/SDA 拉高；
 * 写 0 表示 MCU 主动拉低总线。
 */
static void I2C_Delay(void)
{
    delay_us(5);
}

static void I2C_SCL_Write(uint8_t level)
{
    if (level)
    {
        GPIO_SetBits(SHT30_I2C_SCL_PORT, SHT30_I2C_SCL_PIN);
    }
    else
    {
        GPIO_ResetBits(SHT30_I2C_SCL_PORT, SHT30_I2C_SCL_PIN);
    }
}

static void I2C_SDA_Write(uint8_t level)
{
    if (level)
    {
        GPIO_SetBits(SHT30_I2C_SDA_PORT, SHT30_I2C_SDA_PIN);
    }
    else
    {
        GPIO_ResetBits(SHT30_I2C_SDA_PORT, SHT30_I2C_SDA_PIN);
    }
}

static uint8_t I2C_SDA_Read(void)
{
    return (GPIO_ReadInputDataBit(SHT30_I2C_SDA_PORT, SHT30_I2C_SDA_PIN) == Bit_SET) ? 1u : 0u;
}

static void I2C_Start(void)
{
    I2C_SDA_Write(1);
    I2C_SCL_Write(1);
    I2C_Delay();
    I2C_SDA_Write(0);
    I2C_Delay();
    I2C_SCL_Write(0);
    I2C_Delay();
}

static void I2C_Stop(void)
{
    I2C_SDA_Write(0);
    I2C_SCL_Write(1);
    I2C_Delay();
    I2C_SDA_Write(1);
    I2C_Delay();
}

static uint8_t I2C_WaitAck(void)
{
    uint8_t timeout = 0;

    I2C_SDA_Write(1);
    I2C_Delay();
    I2C_SCL_Write(1);
    I2C_Delay();

    while (I2C_SDA_Read())
    {
        timeout++;
        if (timeout > 50)
        {
            I2C_SCL_Write(0);
            return I2C_NACK;
        }
        delay_us(1);
    }

    I2C_SCL_Write(0);
    I2C_Delay();

    return I2C_ACK;
}

static void I2C_SendAck(uint8_t ack)
{
    I2C_SDA_Write(ack ? 1u : 0u);
    I2C_Delay();
    I2C_SCL_Write(1);
    I2C_Delay();
    I2C_SCL_Write(0);
    I2C_Delay();
    I2C_SDA_Write(1);
}

static uint8_t I2C_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        I2C_SDA_Write((data & 0x80u) ? 1u : 0u);
        I2C_Delay();
        I2C_SCL_Write(1);
        I2C_Delay();
        I2C_SCL_Write(0);
        I2C_Delay();
        data <<= 1;
    }

    return I2C_WaitAck();
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    I2C_SDA_Write(1);

    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        I2C_SCL_Write(1);
        I2C_Delay();
        if (I2C_SDA_Read())
        {
            data |= 1u;
        }
        I2C_SCL_Write(0);
        I2C_Delay();
    }

    I2C_SendAck(ack);
    return data;
}

void BSP_I2C_SHT30_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(SHT30_I2C_SCL_GPIO_CLK |
                           SHT30_I2C_SDA_GPIO_CLK,
                           ENABLE);

    gpio.GPIO_Pin = SHT30_I2C_SCL_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SHT30_I2C_SCL_PORT, &gpio);

    gpio.GPIO_Pin = SHT30_I2C_SDA_PIN;
    GPIO_Init(SHT30_I2C_SDA_PORT, &gpio);

    I2C_SCL_Write(1);
    I2C_SDA_Write(1);
}

uint8_t BSP_I2C_SHT30_Write(uint8_t addr_7bit, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == 0 && len != 0)
    {
        return 0;
    }

    I2C_Start();

    if (I2C_WriteByte((uint8_t)(addr_7bit << 1)) != I2C_ACK)
    {
        I2C_Stop();
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        if (I2C_WriteByte(data[i]) != I2C_ACK)
        {
            I2C_Stop();
            return 0;
        }
    }

    I2C_Stop();
    return 1;
}

uint8_t BSP_I2C_SHT30_Read(uint8_t addr_7bit, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == 0 || len == 0)
    {
        return 0;
    }

    I2C_Start();

    if (I2C_WriteByte((uint8_t)((addr_7bit << 1) | 0x01u)) != I2C_ACK)
    {
        I2C_Stop();
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        data[i] = I2C_ReadByte((i == (len - 1u)) ? I2C_NACK : I2C_ACK);
    }

    I2C_Stop();
    return 1;
}
