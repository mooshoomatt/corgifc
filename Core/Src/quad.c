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
	// DEFAULT RATES
	quad->RATES[0] = X_DEFAULT_RATE;
	quad->RATES[1] = Y_DEFAULT_RATE;
	quad->RATES[2] = Z_DEFAULT_RATE;

	// MEASURED ROTATION
	quad->rot[0] = 0.0;
	quad->rot[1] = 0.0;
	quad->rot[2] = 0.0;

	// SETPOINT
	quad->set[0] = 0.0;
	quad->set[1] = 0.0;
	quad->set[2] = 0.0;

	// RATE BUFFERS
	quad->gyro_rate[0] = 0.0;
	quad->gyro_rate[1] = 0.0;
	quad->gyro_rate[2] = 0.0;
	quad->stick_rate[0] = 0.0;
	quad->stick_rate[1] = 0.0;
	quad->stick_rate[2] = 0.0;

	// GET FIRST TIME MEASUREMENT
	quad->tprev = __HAL_TIM_GET_COUNTER(quad->htim);
}

/* UPDATE PROCEDURE */
void QUAD_UPDATE(QUAD *quad, volatile uint16_t *IC)
{
	// CHECK IF ARMED

	// READ GYROSCOPE
	if ( BMI088_I2C_Read_Gyro(quad->hi2c, quad->gyro_buf) != HAL_OK ) { Error_Handler(); }

	// UPDATE TIMER
	quad->telapsed = __HAL_TIM_GET_COUNTER(quad->htim) - quad->tprev;
	quad->tprev    = quad->tprev + quad->telapsed;

	// CONVERT TO SIGNED INTEGER, SCALE, AND INTEGRATE
	for (int i = 0; i < 3; i++)
	{
		quad->temp         = quad->gyro_buf[2*i + 1] << 8 | quad->gyro_buf[2*i];
		quad->gyro_rate[i] = ((float)quad->temp*GYRO_RATE*M_PI)/(32767.0*180.0);
		quad->rot[i]       = quad->rot[i] + 0.000001*(float)quad->telapsed*quad->gyro_rate[i];
	}

	// UPDATE ROTATION SETPOINT
	for (int i = 0; i < 3; i++)
	{
		quad->stick_rate[i] = (float)IC[i]*quad->RATES[i]/IC_SCALE;
		quad->set[i]        = quad->set[i] + 0.000001*(float)quad->telapsed*quad->stick_rate[i];
	}

	// ITERATE PID ALGORITHM
	if (PID3_Update(quad->PID, quad->set, quad->rot, 0.000001*(float)quad->telapsed) != PID_OK) { Error_Handler(); }

	// UPDATE MOTOR SETTINGS
	if (CommandFromSetpoint(quad->OS) != OS125_OK) { Error_Handler(); }
	if (UpdateCCR(quad->OS) != OS125_OK) { Error_Handler(); }
}

/* SEND ORIENTATION OVER SERIAL */
void QUAD_SEND_ORIENTATION(QUAD *quad)
{
	// SEND ORIENTATION DATA OVER VIRTUAL COM PORT
	// DATA FORMAT: [X ANGLE]    [Y ANGLE]    [Z ANGLE]    [OTHER]
	sprintf(quad->tx_buf, "%f\t%f\t%f\t%f\t%f\t%f\t%i\n", quad->rot[0], quad->rot[1], quad->rot[2], quad->PID->out[0], quad->PID->out[1], quad->PID->out[2], quad->telapsed);
	CDC_Transmit_FS((uint8_t*)(quad->tx_buf), strlen(quad->tx_buf));
}
