/*
 * quad.c
 *
 *  Created on: Feb 21, 2021
 *      Author: Matthew Morawiec
 */


#include "quad.h"

/* INITIALIZE QUAD ROTOR STRUCT */
void QUAD_Init(QUAD *quad)
{
	// SET DEFAULT RATES
	quad->RATES[0] = X_DEFAULT_RATE; quad->RATES[1] = Y_DEFAULT_RATE; quad->RATES[2] = Z_DEFAULT_RATE;

	// CLEAR MEASURED ROTATION
	quad->rot[0] = 0.0; quad->rot[1] = 0.0; quad->rot[2] = 0.0;

	// CLEAR SETPOINT
	quad->set[0] = 0.0; quad->set[1] = 0.0; quad->set[2] = 0.0;

	// SET NORMAL OPERATION MODE
	quad->TEST_MODE = 0x0;

	// GET FIRST TIME MEASUREMENT
	quad->tprev = __HAL_TIM_GET_COUNTER(quad->htim);
}

/* CLEAR STATE FUNCTION */
void QUAD_Clear(QUAD *quad)
{
	// CLEAR MEASURED ROTATION
	quad->rot[0] = 0.0; quad->rot[1] = 0.0; quad->rot[2] = 0.0;

	// CLEAR SETPOINT
	quad->set[0] = 0.0; quad->set[1] = 0.0; quad->set[2] = 0.0;
}

/* UPDATE PROCEDURE */
void QUAD_UPDATE(QUAD *quad)
{
	// UPDATE TIMER
	quad->telapsed = __HAL_TIM_GET_COUNTER(quad->htim) - quad->tprev;
	quad->tprev    = quad->tprev + quad->telapsed;

	// CHECK IF ARMED
	if (quad->IC->elapsed[ARM_CHANNEL] > 1500)
	{
		// SET ARM_STATUS FLAG
		quad->ARM_STATUS = 0x1;

		// READ GYROSCOPE
		if ( BMI088_I2C_Read_Gyro(quad->hi2c, quad->gyro_buf) != HAL_OK ) { Error_Handler(); }

		// CONVERT TO SIGNED INTEGER, SCALE, AND INTEGRATE
		for (int i = 0; i < 3; i++)
		{
			quad->temp_i       = quad->gyro_buf[2*i + 1] << 8 | quad->gyro_buf[2*i];
			quad->gyro_rate[i] = ((float)quad->temp_i*GYRO_RATE*M_PI)/(32767.0*180.0);
			quad->rot[i]       = quad->rot[i] + 0.000001*(float)quad->telapsed*quad->gyro_rate[i];
		}

		// UPDATE ROTATION SETPOINT
		for (int i = 0; i < 3; i++)
		{
			quad->stick_rate[i] = quad->RATES[i]*((float)quad->IC->elapsed[i+1] - 1500.0)/500.0;
			quad->set[i]        = quad->set[i] + 0.000001*(float)quad->telapsed*quad->stick_rate[i];
		}

		// ITERATE PID ALGORITHM
		if (PID3_Update(quad->PID, quad->set, quad->rot, 0.000001*(float)quad->telapsed) != PID_OK) { Error_Handler(); }

		// CHECK IF WE ARE IN TEST MODE
		if (quad->TEST_MODE == 0x1) { OS125_ThrottlePassThrough(quad->OS); }
		// OTHERWISE CALCULATE COMMAND FROM PID OUTPUT
		else						{ OS125_CommandFromSetpoint(quad->OS); }

		// UPDATE THE MOTOR OUTPUTS
		OS125_UpdateCCR(quad->OS);
	}
	// NOT ARMED
	else
	{
		// SET ARM_STATUS FLAG
		quad->ARM_STATUS = 0x0;

		// CLEAR INTERNAL STATE
		QUAD_Clear(quad);
		PID3_Clear(quad->PID);
		OS125_Disarm(quad->OS);
	}

}

/* SEND ORIENTATION OVER SERIAL */
void QUAD_SEND_ORIENTATION(QUAD *quad)
{
	// SEND ORIENTATION DATA OVER VIRTUAL COM PORT
	// DATA FORMAT: [X ANGLE]    [Y ANGLE]    [Z ANGLE]    [OTHER]
	sprintf(quad->tx_buf, "%f %f %f %f %f %f %f %f %f %lu %lu %lu %lu\n",
			quad->set[0], quad->set[1], quad->set[2],
			quad->rot[0], quad->rot[1], quad->rot[2],
			quad->PID->out[0], quad->PID->out[1], quad->PID->out[2],
			quad->OS->TIM->CCR1, quad->OS->TIM->CCR2, quad->OS->TIM->CCR3, quad->OS->TIM->CCR4);
	CDC_Transmit_FS((uint8_t*)(quad->tx_buf), strlen(quad->tx_buf));

	// ALSO CHECK IF WE NEED TO UPDATE ARM_STATUS LED
	if (quad->ARM_STATUS == 1) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); }
	else { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); }
}
