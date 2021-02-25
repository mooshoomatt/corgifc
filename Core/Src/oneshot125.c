/*
 * oneshot125.c
 *
 *  Created on: Feb 20, 2021
 *      Author: Matthew Morawiec
 */

#include "oneshot125.h"

/*
 * ONESHOT125 OUTPUT INITIALIZATION FUNCTION
 * Prerequisites:
 * 1) Set OS.TIM
 * 2) Set OS.command
 * 3) Set OS.fclk
 * 4) Set OS.fclk_psc
 */
OS125_StatusTypeDef OS125_Init(ONESHOT125 *OS)
{
	// GET TIMER INSTANCE
	OS->TIM = OS->htim->Instance;
	// CALCULATE TIMER CLOCK FREQUENCY
	OS->tim_clk = (float)OS->fclk / ((float)OS->fclk_psc + 1);
	// CALCULATE TIMER CLOCK PERIOD
	OS->dt = 1.0/OS->tim_clk;
	// CALCULATE MAXIMUM CCR VALUE (100% Power)
	OS->CCR_MAX = (int)ceil(0.000250 / OS->dt);
	// CALCULATE MINIMUM CCR VALUE (0% Power)
	OS->CCR_MIN = (int)floor(0.000125 / OS->dt);
	// MINIMUM IDLE CCR VALUE
	OS->CCR_MIN_ON = (int)OS->CCR_MIN*(1.0 + MIN_ON_MULTIPLIER);
	// CALCULATE NUMBER OF CCR STEPS
	OS->CCR_STEPS = OS->CCR_MAX - OS->CCR_MIN;

	// SET ALL OUTPUTS TO 0
	for (int i = 0; i < 4; i++) { OS->CCR[i] = OS->CCR_MIN; }
	OS125_UpdateCCR(OS);

	return OS125_OK;
}

/*
 * ONESHOT125 OUTPUT UPDATE FUNCTION
 * Converts PID controller output to PWM motor commands
 */
OS125_StatusTypeDef OS125_CommandFromSetpoint(ONESHOT125 *OS)
{
	// {TODO}: TAKE IN INPUT CAPTURE DATA AND PID OUTPUT AND
	//         CONVERT TO CCR VALUES (PWM OUTPUTS)

	// READ THROTTLE VALUE (MIN_ON - 1 scale)
	OS->throttle = fmin(1.0, fmax(MIN_ON_MULTIPLIER, ((float)OS->IC[0]-1000.0)/1000.0));

	// CALCULATE MOTOR SPEEDS AND CONVERT TO CCR
	// MOTOR 1 (TOP RIGHT)
	OS->CCR[0] = fmin(OS->CCR_MAX, fmax(OS->CCR_MIN_ON, OS->CCR_MIN + (int)OS->CCR_STEPS*(OS->throttle - OS->com[0] - OS->com[1] - OS->com[2])));
	// MOTOR 2 (TOP LEFT)
	OS->CCR[1] = fmin(OS->CCR_MAX, fmax(OS->CCR_MIN_ON, OS->CCR_MIN + (int)OS->CCR_STEPS*(OS->throttle + OS->com[0] - OS->com[1] + OS->com[2])));
	// MOTOR 3 (BOTTOM LEFT)
	OS->CCR[2] = fmin(OS->CCR_MAX, fmax(OS->CCR_MIN_ON, OS->CCR_MIN + (int)OS->CCR_STEPS*(OS->throttle + OS->com[0] + OS->com[1] - OS->com[2])));
	// MOTOR 4 (BOTTOM RIGHT)
	OS->CCR[3] = fmin(OS->CCR_MAX, fmax(OS->CCR_MIN_ON, OS->CCR_MIN + (int)OS->CCR_STEPS*(OS->throttle - OS->com[0] + OS->com[1] + OS->com[2])));

	return OS125_OK;
}

/*
 * ONESHOT125 SET OUTPUT DIRECT FUNCTION
 * Updates timer CCR registers according to internal CCR array
 */
OS125_StatusTypeDef OS125_UpdateCCR(ONESHOT125 *OS)
{
	OS->TIM->CCR1 = OS->CCR[0];
	OS->TIM->CCR2 = OS->CCR[1];
	OS->TIM->CCR3 = OS->CCR[2];
	OS->TIM->CCR4 = OS->CCR[3];

	return OS125_OK;
}

/*
 * ONESHOT125 SET OUTPUT DIRECT FUNCTION
 * Sets all CCR registers
 */
OS125_StatusTypeDef OS125_SetCCR(ONESHOT125 *OS, int setpoint)
{
	if ((setpoint < OS->CCR_MIN) || (setpoint > OS->CCR_MAX))
	{
		return OS125_ERROR;
	}

	OS->TIM->CCR1 = setpoint;
	OS->TIM->CCR2 = setpoint;
	OS->TIM->CCR3 = setpoint;
	OS->TIM->CCR4 = setpoint;

	return OS125_OK;
}

/*
 * ONESHOT125 ALL OUTPUTS OFF FUNCTION
 * ENSURES THAT ALL MOTORS ARE OFF
 */
OS125_StatusTypeDef OS125_Disarm(ONESHOT125 *OS)
{
	OS->TIM->CCR1 = OS->CCR_MIN;
	OS->TIM->CCR2 = OS->CCR_MIN;
	OS->TIM->CCR3 = OS->CCR_MIN;
	OS->TIM->CCR4 = OS->CCR_MIN;

	return OS125_OK;
}
