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

/* CUSTOM GYROSCOPE SETTING INITIALIZATION */
HAL_StatusTypeDef BMI088_I2C_GYRO_INIT(I2C_HandleTypeDef *hi2c)
{
	/* FOR REGISTERS WITH RESERVED BITS, DATASHEET RECOMMENDS READING THE REGISTER
	 * FIRST AND THEN MODIFYING THE CONTENTS USING BITWISE OPERATIONS, BEFORE
	 * WRITING BACK TO THE REGISTER
	 */

	HAL_StatusTypeDef ret;
	uint8_t buf[1]; // Tx/Rx Buffer
	ret = HAL_OK;

	// SET GYROSCOPE RANGE:
	// buf[0] = GYRO_RANGE_SETTING;
	// ret = (ret | BMI088_I2C_Reg_Write(hi2c, BMI088_GYRO_ADDR, GYRO_RANGE, buf));

	// SET GYROSCOPE DATA RATE AND BANDWIDTH
	// DATA RATE: 2000 HZ    1000 HZ    [400 HZ]
	// BANDWIDTH: 532 HZ    230 HZ    116 Hz    [47 HZ]
	buf[0] = 0x81;
	ret    |= BMI088_I2C_Reg_Write(hi2c, BMI088_GYRO_ADDR, GYRO_BANDWIDTH, buf);

	// SET GYROSCOPE INT3 MODE TO PUSH-PULL
	ret    |= BMI088_I2C_Reg_Read(hi2c, BMI088_GYRO_ADDR, INT3_INT4_IO_CONF, buf);
	buf[0] &= 0xFD;
	ret    |= BMI088_I2C_Reg_Write(hi2c, BMI088_GYRO_ADDR, INT3_INT4_IO_CONF, buf);

	// MAP GYROSCOPE DATA-READY INTERRUPT TO INT3
	ret    |= BMI088_I2C_Reg_Read(hi2c, BMI088_GYRO_ADDR, INT3_INT4_IO_MAP, buf);
	buf[0] |= 0x01;
	ret    |= BMI088_I2C_Reg_Write(hi2c, BMI088_GYRO_ADDR, INT3_INT4_IO_MAP, buf);

	// ENABLE GYROSCOPE DATA-READY INTERRUPT
	ret    |= BMI088_I2C_Reg_Read(hi2c, BMI088_GYRO_ADDR, GYRO_INT_CTRL, buf);
	buf[0] |= 0x80;
	ret    |= BMI088_I2C_Reg_Write(hi2c, BMI088_GYRO_ADDR, GYRO_INT_CTRL, buf);

	return ret;
}

/* CUSTOM ACCELEROMETER SETTING INITIALIZATION */
HAL_StatusTypeDef BMI088_I2C_ACCEL_INIT(I2C_HandleTypeDef *hi2c)
{
	/* FOR REGISTERS WITH RESERVED BITS, DATASHEET RECOMMENDS READING THE REGISTER
	 * FIRST AND THEN MODIFYING THE CONTENTS USING BITWISE OPERATIONS, BEFORE
	 * WRITING BACK TO THE REGISTER
	 */

	HAL_StatusTypeDef ret;
	uint8_t buf[1]; // Tx/Rx Buffer
	ret = HAL_OK;

	// SET ACCELEROMETER MODE TO NORMAL
	buf[0] = ACC_MODE_NORMAL;
	ret = (ret | BMI088_I2C_Reg_Write(hi2c, BMI088_ACC_ADDR, ACC_PWR_CTRL, buf));

	// SET ACCELEROMETER RANGE

	// SET ACCELEROMETER DATA RATE AND BANDWIDTH

	return ret;
}

/* READ ALL ACCELEROMETER DATA IN POLLING MODE */
HAL_StatusTypeDef BMI088_I2C_Read_Accel(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	return HAL_I2C_Mem_Read(hi2c, BMI088_ACC_ADDR << 1, ACC_DATA, I2C_MEMADD_SIZE_8BIT, pData, 6, HAL_MAX_DELAY);
}

/* READ ALL ACCELEROMETER DATA IN INTERRUPT MODE */
HAL_StatusTypeDef BMI088_I2C_Read_Accel_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	return HAL_I2C_Mem_Read_IT(hi2c, BMI088_ACC_ADDR << 1, ACC_DATA, I2C_MEMADD_SIZE_8BIT, pData, 6);
}

/* READ ALL GYROSCOPE DATA IN POLLING MODE */
HAL_StatusTypeDef BMI088_I2C_Read_Gyro(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	return HAL_I2C_Mem_Read(hi2c, BMI088_GYRO_ADDR << 1, GYRO_DATA, I2C_MEMADD_SIZE_8BIT, pData, 6, HAL_MAX_DELAY);
}

/* READ ALL GYROSCOPE DATA IN INTERRUPT MODE */
HAL_StatusTypeDef BMI088_I2C_Read_Gyro_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	return HAL_I2C_Mem_Read_IT(hi2c, BMI088_GYRO_ADDR << 1, GYRO_DATA, I2C_MEMADD_SIZE_8BIT, pData, 6);
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
