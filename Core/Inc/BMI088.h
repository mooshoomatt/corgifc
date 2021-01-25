/*
 * BMI088.h
 *
 *  Created on: Jan 24, 2021
 *      Author: matth
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* I2C ADDRESSES */
#define BMI088_ACCEL_ADDR 0x18
#define BMI088_GYRO_ADDR  0x68

/* ACCELEROMETER REGISTERS */
#define ACC_CHIP_ID    0x00
#define ACC_ERR_REG    0x02
#define ACC_STATUS     0x03
#define ACC_DATA       0x12
#define ACC_INT_STAT_1 0x1D
#define ACC_CONF       0x40
#define ACC_RANGE      0x41
#define ACC_SELF_TEST  0x6D
#define ACC_PWR_CONF   0x7C
#define ACC_PWR_CTRL   0x7D
#define ACC_SOFTRESET  0x7E

/* GYROSCOPE REGISTERS */
#define GYRO_CHIP_ID    0x00
#define GYRO_DATA       0x02
#define GYRO_INT_STAT_1 0x0A
#define GYRO_RANGE      0x0F
#define GYRO_BANDWIDTH  0x10
#define GYRO_LPM1       0x11
#define GYRO_SOFTRESET  0x14
#define GYRO_INT_CTRL   0x15
#define GYRO_SELF_TEST  0x3C

/* INTERRUPT CONFIG REGISTERS */
#define INT1_IO_CONF       0x53
#define INT2_IO_CONF       0x54
#define INT1_INT2_MAP_DATA 0x58
#define INT3_INT4_IO_CONF  0x16
#define INT3_INT4_IO_MAP   0x18

/* READ ALL ACCELEROMETER DATA */
HAL_StatusTypeDef BMI088_I2C_Read_Accel(I2C_HandleTypeDef *hi2c, uint8_t *pData);

/* READ ALL GYROSCOPE DATA */
HAL_StatusTypeDef BMI088_I2C_Read_Gyro(I2C_HandleTypeDef *hi2c, uint8_t *pData);

#endif /* INC_BMI088_H_ */
