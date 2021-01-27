/*
 * BMI088.c
 *
 *  Created on: Jan 24, 2021
 *      Author: Matthew Morawiec
 */

#include "BMI088.h"

/* READ ACCEL AND GYRO CHIP IDS */
HAL_StatusTypeDef BMI088_I2C_Read_Accel_ID(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret; // HAL Status Value
	uint8_t buf[4];        // Tx/Rx Buffer

	ret    = HAL_OK;
	buf[0] = ACC_CHIP_ID;
    ret = (ret | HAL_I2C_Master_Transmit(hi2c, BMI088_ACC_ADDR << 1, buf, 1, HAL_MAX_DELAY));
	ret = (ret | HAL_I2C_Master_Receive (hi2c, BMI088_ACC_ADDR << 1, buf, 1, HAL_MAX_DELAY));
	if (buf[0] != BMI088_ACC_ID){ ret = HAL_ERROR; }
	buf[0] = GYRO_CHIP_ID;
	ret = (ret | HAL_I2C_Master_Transmit(hi2c, BMI088_GYRO_ADDR << 1, buf, 1, HAL_MAX_DELAY));
	ret = (ret | HAL_I2C_Master_Receive (hi2c, BMI088_GYRO_ADDR << 1, buf, 1, HAL_MAX_DELAY));
	if (buf[0] != BMI088_GYRO_ID){ ret = HAL_ERROR; }

	return ret;
}

/* CUSTOM SETTING INITIALIZATION */
HAL_StatusTypeDef BMI088_I2C_CORGI_INIT(I2C_HandleTypeDef *hi2c)
{

}

/* READ ALL ACCELEROMETER DATA */
HAL_StatusTypeDef BMI088_I2C_Read_Accel(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	HAL_StatusTypeDef ret; // HAL Status Value
	uint8_t addr[1];       // Slave Address Buffer

	ret     = HAL_OK;
	addr[0] = ACC_DATA;
	ret = (ret | HAL_I2C_Master_Transmit(hi2c, BMI088_ACC_ADDR << 1, addr,  1, HAL_MAX_DELAY));
	ret = (ret | HAL_I2C_Master_Receive (hi2c, BMI088_ACC_ADDR << 1, pData, 6, HAL_MAX_DELAY));

	return ret;
}

/* READ ALL GYROSCOPE DATA */
HAL_StatusTypeDef BMI088_I2C_Read_Gyro(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	HAL_StatusTypeDef ret; // HAL Status Value
	uint8_t addr[1];       // Slave Address Buffer

	ret     = HAL_OK;
	addr[0] = GYRO_DATA;
	ret = (ret | HAL_I2C_Master_Transmit(hi2c, BMI088_GYRO_ADDR << 1, addr,  1, HAL_MAX_DELAY));
	ret = (ret | HAL_I2C_Master_Receive (hi2c, BMI088_GYRO_ADDR << 1, pData, 6, HAL_MAX_DELAY));

	return ret;
}
