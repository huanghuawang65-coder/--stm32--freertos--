#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * bsp_i2c.h
 *
 * V2 硬件中 SHT30 使用 PC0/PC1 软件 I2C。
 * 这里不占用 STM32 硬件 I2C1，因为 PB6/PB7 已经分配给电机编码器 TIM4。
 */

void BSP_I2C_SHT30_Init(void);
uint8_t BSP_I2C_SHT30_Write(uint8_t addr_7bit, const uint8_t *data, uint8_t len);
uint8_t BSP_I2C_SHT30_Read(uint8_t addr_7bit, uint8_t *data, uint8_t len);

#endif /* __BSP_I2C_H */
