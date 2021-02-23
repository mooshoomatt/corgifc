/*
 * PID1.h
 *
 * SINGLE AXIS PID CONTROLLER
 *
 *  Created on: Feb 7, 2021
 *      Author: Matthew Morawiec
 */

#ifndef INC_PID1_H_
#define INC_PID1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define PID_OK    0x00
#define PID_ERROR 0x01

#define PID_MAXINT  1000.0;
#define PID_MININT -1000.0;
#define PID_MAXOUT  1000.0;
#define PID_MINOUT -1000.0;

typedef uint8_t PID_StatusTypeDef;

typedef struct {

	/* SETPOINT, MEASUREMENT, AND OUTPUT */
	float set;
	float meas;
	float output;

	/* GAIN VALUES */
	float Kp;
	float Ki;
	float Kd;

	/* INTEGRATOR WIND-UP LIMITS */
	float intLimMax;
	float intLimMin;

	/* ABSOLUTE OUTPUT LIMITS */
	float outLimMax;
	float outLimMin;

	/* D-TERM LOW-PASS TIME CONSTANT */
	float tau;

	/* ERROR TEMPVAR */
	float error;

	/* PROPORTIONAL TEMPVAR */
	float proportional;

	/* INTEGRATOR TEMPVARS*/
	float integral;
	float prevError;

	/* DIFFERENTIATOR TEMPVARS */
	float derivative;
	float prevMeas;

} PID1;

/* INITIALIZATION FUNCTION */
PID_StatusTypeDef PID1_Init(PID1 *pid, const float p, const float i, const float d);

/* SET FILTER TIME CONSTANT */
PID_StatusTypeDef PID1_Set_Tau(PID1 *pid, float t);

/* SET INTEGRATOR LIMIT */
PID_StatusTypeDef PID1_Set_Integrator_Limit(PID1 *pid, float min, float max);

/* SET OUTPUT LIMIT */
PID_StatusTypeDef PID1_Set_Output_Limit(PID1 *pid, float min, float max);

/* ADVANCE TIMESTEP FUNCTION */
PID_StatusTypeDef PID1_Update(PID1 *pid, float T);

#endif /* INC_PID1_H_ */
