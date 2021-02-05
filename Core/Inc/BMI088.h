/*
 * BMI088.h
 *
 *  Created on: Jan 24, 2021
 *      Author: Matthew Morawiec
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* I2C ADDRESSES */
#define BMI088_ACC_ADDR        0x18
#define BMI088_GYRO_ADDR       0x68

/* CHIP IDs */
#define BMI088_ACC_ID          0x1E
#define BMI088_GYRO_ID         0x0F

/* ACCELEROMETER REGISTERS */
#define ACC_CHIP_ID            0x00
#define ACC_ERR_REG            0x02
#define ACC_STATUS             0x03
#define ACC_DATA               0x12
#define ACC_INT_STAT_1         0x1D
#define ACC_CONF               0x40
#define ACC_RANGE              0x41
#define ACC_SELF_TEST          0x6D
#define ACC_PWR_CONF           0x7C
#define ACC_PWR_CTRL           0x7D
#define ACC_SOFTRESET          0x7E

/* GYROSCOPE REGISTERS */
#define GYRO_CHIP_ID           0x00
#define GYRO_DATA              0x02
#define GYRO_INT_STAT_1        0x0A
#define GYRO_RANGE             0x0F
#define GYRO_BANDWIDTH         0x10
#define GYRO_LPM1              0x11
#define GYRO_SOFTRESET         0x14
#define GYRO_INT_CTRL          0x15
#define GYRO_SELF_TEST         0x3C

/* INTERRUPT CONFIG REGISTERS */
#define INT1_IO_CONF           0x53
#define INT2_IO_CONF           0x54
#define INT1_INT2_MAP_DATA     0x58
#define INT3_INT4_IO_CONF      0x16
#define INT3_INT4_IO_MAP       0x18

/* ACCELEROMETER POWER MODES */
#define ACC_MODE_NORMAL        0x04
#define ACC_MODE_SUSPEND       0x00

/* GYROSCOPE POWER MODES */
#define GYRO_MODE_NORMAL       0x00
#define GYRO_MODE_SUSPEND      0x80
#define GYRO_MODE_DEEP_SUSPEND 0x20

#define DATA_READY             0x01
#define DATA_RESET             0x00

/* READ ACCEL AND GYRO CHIP IDS */
HAL_StatusTypeDef BMI088_I2C_Read_CHIP_IDS(I2C_HandleTypeDef *hi2c);

/* CUSTOM GYROSCOPE SETTING INITIALIZATION */
HAL_StatusTypeDef BMI088_I2C_GYRO_INIT(I2C_HandleTypeDef *hi2c);

/* CUSTOM ACCELEROMETER SETTING INITIALIZATION */
HAL_StatusTypeDef BMI088_I2C_ACCEL_INIT(I2C_HandleTypeDef *hi2c);

/* READ ALL ACCELEROMETER DATA IN POLLING MODE */
HAL_StatusTypeDef BMI088_I2C_Read_Accel(I2C_HandleTypeDef *hi2c, uint8_t *pData);

/* READ ALL ACCELEROMETER DATA IN INTERRUPT MODE */
HAL_StatusTypeDef BMI088_I2C_Read_Accel(I2C_HandleTypeDef *hi2c, uint8_t *pData);

/* READ ALL GYROSCOPE DATA IN POLLING MODE */
HAL_StatusTypeDef BMI088_I2C_Read_Gyro(I2C_HandleTypeDef *hi2c, uint8_t *pData);

/* READ ALL GYROSCOPE DATA IN INTERRUPT MODE*/
HAL_StatusTypeDef BMI088_I2C_Read_Gyro_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData);

/* WRITE REGISTER */
HAL_StatusTypeDef BMI088_I2C_Reg_Write(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t mem_addr, uint8_t *data);

/* READ REGISTER AND PRINT TO CONSOLE */
HAL_StatusTypeDef BMI088_I2C_Reg_Read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t mem_addr, uint8_t *pData);

#endif /* INC_BMI088_H_ */
