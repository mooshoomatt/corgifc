/*
 * PID3.h
 *
 * 3-AXIS PID CONTROLLER
 *
 *  Created on: Feb 7, 2021
 *      Author: Matthew Morawiec
 */

#ifndef INC_PID3_H_
#define INC_PID3_H_

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

	/* SETPOINT, ORIENTATION, AND OUTPUT */
	float set[3];
	float rot[3];
	float out[3];

	/* GAIN VALUES */
	float Kp[3];
	float Ki[3];
	float Kd[3];

	/* SAMPLING PERIOD (SECONDS) */
	float T;

	/* INTEGRATOR WIND-UP LIMITS */
	float intMaxLim;
	float intMinLim;

	/* ABSOLUTE OUTPUT LIMITS */
	float outMaxLim;
	float outMinLim;

	/* D-TERM LOW-PASS TIME CONSTANT */
	float tau;

	/* ERROR TEMPVAR */
	float error[3];

	/* TEMPVARS FOR INTEGRATOR */
	float integral[3];
	float prevError[3];

	/* TEMPVARS FOR DIFFERENTIATOR */
	float derivative[3];
	float prevMeas[3];

} PID3;

/* INITIALIZATION FUNCTION */
PID_StatusTypeDef PID3_Init(PID3 *pid, float *p, float *i, float*d);

/* SET INTEGRATOR LIMIT */
PID_StatusTypeDef PID3_Set_Integrator_Limit(PID3 *pid, float min, float max);

/* SET OUTPUT LIMIT */
PID_StatusTypeDef PID3_Set_Output_Limit(PID3 *pid, float min, float max);

/* ADVANCE TIMESTEP FUNCTION */
PID_StatusTypeDef PID3_Update(PID3 *pid, float T);

#endif /* INC_PID3_H_ */
