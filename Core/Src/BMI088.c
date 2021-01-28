/*
 * BMI088.c
 *
 *  Created on: Jan 24, 2021
 *      Author: Matthew Morawiec
 */

#include "BMI088.h"

/* READ ACCEL AND GYRO CHIP IDS */
HAL_StatusTypeDef BMI088_I2C_Read_CHIP_IDS(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret; // HAL Status Value
	uint8_t buf[4];        // Rx Buffer

	ret    = HAL_OK;
    ret = (ret | HAL_I2C_Mem_Read(hi2c, BMI088_ACC_ADDR << 1, ACC_CHIP_ID, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY));
	if ( buf[0] != BMI088_ACC_ID ){ ret = HAL_ERROR; }
	ret = (ret | HAL_I2C_Mem_Read(hi2c, BMI088_GYRO_ADDR << 1, GYRO_CHIP_ID, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY));
	if ( buf[0] != BMI088_GYRO_ID ){ ret = HAL_ERROR; }

	return ret;
}

/* CUSTOM SETTING INITIALIZATION */
HAL_StatusTypeDef BMI088_I2C_CORGI_INIT(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef ret;
	//uint8_t buf[1]; // Tx Buffer
	ret = HAL_OK;

	// SET ACCELEROMETER MODE: NORMAL
	// buf[0] = ACC_MODE_NORMAL;
	// ret = (ret | BMI088_I2C_Reg_Write(hi2c, BMI088_ACC_ADDR, ACC_PWR_CTRL, buf));

	// SET GYROSCOPE MODE:

	// SET ACCELEROMETER RANGE:

	// SET GYROSCOPE RANGE:

	// SET ACCELEROMETER LPF:

	// SET GYROSCOPE LPF:

	return ret;
}

/* READ ALL ACCELEROMETER DATA */
HAL_StatusTypeDef BMI088_I2C_Read_Accel(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	return HAL_I2C_Mem_Read(hi2c, BMI088_ACC_ADDR << 1, ACC_DATA, I2C_MEMADD_SIZE_8BIT, pData, 6, HAL_MAX_DELAY);
}

/* READ ALL GYROSCOPE DATA */
HAL_StatusTypeDef BMI088_I2C_Read_Gyro(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	return HAL_I2C_Mem_Read(hi2c, BMI088_GYRO_ADDR << 1, GYRO_DATA, I2C_MEMADD_SIZE_8BIT, pData, 6, HAL_MAX_DELAY);
}

/* WRITE REGISTER */
HAL_StatusTypeDef BMI088_I2C_Reg_Write(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t mem_addr, uint8_t *pData)
{
	return HAL_I2C_Mem_Write(hi2c, dev_addr << 1, mem_addr, I2C_MEMADD_SIZE_8BIT, pData, 1, HAL_MAX_DELAY);
}

/* READ REGISTER */
HAL_StatusTypeDef BMI088_I2C_Reg_Read(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t mem_addr, uint8_t *pData)
{
	return HAL_I2C_Mem_Read(hi2c, dev_addr << 1, mem_addr, I2C_MEMADD_SIZE_8BIT, pData, 1, HAL_MAX_DELAY);
}
