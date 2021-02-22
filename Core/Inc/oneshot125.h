/*
 * oneshot125.h
 *
 *  Created on: Feb 20, 2021
 *      Author: Matthew Morawiec
 *
 *  ONESHOT125 OUTPUT IMPLEMENTATION
 *  PWM SCHEME FOR CONTROLLING ELECTRONIC SPEED CONTROLLERS
 *
 *  0% OUTPUT   = 125 US PULSE WIDTH
 *  100% OUTPUT = 250 US PULSE WIDTH
 */

#ifndef INC_ONESHOT125_H_
#define INC_ONESHOT125_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"

#define OS125_OK    0x00
#define OS125_ERROR 0x01

typedef uint8_t OS125_StatusTypeDef;

/*
 * 4 Channel OneShot125 Output
 * OUTPUT FREQUENCY: 2000 HZ
 * 0%   POWER -> 125 US PULSE WIDTH
 * 100% POWER -> 250 US PULSE WIDTH
 */
typedef struct {


	TIM_HandleTypeDef *htim; // TIMER HANDLE
	TIM_TypeDef       *TIM;  // TIMER INSTANCE

	float *command;          // COMMAND FLOAT ARRAY
	int   CCR[4];            // MOTOR OUTPUT ARRAY

	/* TIMER INFORMATION */
	int   fclk;     // TIMER SOURCE CLOCK FREQUENCY
	int   fclk_psc; // TIMER CLOCK PRESCALER

	float tim_clk;  // ACTUAL TIMER CLOCK FREQUENCY
	float dt;       // ACTUAL TIMER CLOCK PERIOD

	int CCR_MAX;    // CCR VALUE CORRESPONDING TO 100% POWER
	int CCR_MIN;    // CCR VALUE CORRESPONDING TO 0% POWER
	int CCR_STEPS;  // NUMBER OF STEPS

} ONESHOT125;

/*
 * ONESHOT125 OUTPUT INITIALIZATION FUNCTION
 * Calculates important frequencies and scaling factors
 *
 * Prerequisites:
 * 1) Set OS.htim
 * 2) Set OS.command
 * 3) Set OS.fclk
 * 4) Set OS.fclk_psc
 */
OS125_StatusTypeDef OS125_Init(ONESHOT125 *OS);

/*
 * ONESHOT125 OUTPUT UPDATE FUNCTION
 * Converts PID controller output to PWM motor commands
 */
OS125_StatusTypeDef CommandFromSetpoint(ONESHOT125 *OS);

/*
 * ONESHOT125 SET OUTPUT DIRECT FUNCTION
 * Updates timer CCR registers according to internal CCR array
 */
OS125_StatusTypeDef UpdateCCR(ONESHOT125 *OS);

/*
 * ONESHOT125 SET OUTPUT DIRECT FUNCTION
 * Sets all 4 CCR registers according to setpoint
 */
OS125_StatusTypeDef SetCCR(ONESHOT125 *OS, int setpoint);

#endif /* INC_ONESHOT125_H_ */
