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
#include <math.h>

#define PID_OK    0x00
#define PID_ERROR 0x01

#define PID_MAXINT  1000.0;
#define PID_MININT -1000.0;
#define PID_MAXOUT  1000.0;
#define PID_MINOUT -1000.0;

typedef uint8_t PID_StatusTypeDef;

typedef struct {

	/* SETPOINT, ORIENTATION, AND OUTPUT */
	float out[3];
  //float rot[3] is accessible via the quad struct as quad.rot
  //float set[3] is accessible via the quad struct as quad.set

	/* GAIN VALUES */
	float Kp[3];
	float Ki[3];
	float Kd[3];

	/* INTEGRATOR WIND-UP LIMITS */
	float intLimMax;
	float intLimMin;

	/* ABSOLUTE OUTPUT LIMITS */
	float outLimMax;
	float outLimMin;

	/* D-TERM LOW-PASS TIME CONSTANT */
	float tau;

	/* ERROR TEMPVAR */
	float error[3];

	/* PROPORTIONAL TEMPVAR */
	float proportional[3];

	/* TEMPVARS FOR INTEGRATOR */
	float integral[3];
	float prevError[3];

	/* TEMPVARS FOR DIFFERENTIATOR */
	float derivative[3];
	float prevRot[3];

  //float SAMPLING_PERIOD is accessible via the quad struct as quad.telapsed

} PID3;

/* INITIALIZATION FUNCTION */
PID_StatusTypeDef PID3_Init(PID3 *pid, const float *p, const float *i, const float*d);

/* CLEAR STATE FUNCTION */
PID_StatusTypeDef PID3_Clear(PID3 *pid);

/* SET FILTER TIME CONSTANT */
PID_StatusTypeDef PID3_Set_Tau(PID3 *pid, float t);

/* SET INTEGRATOR LIMIT */
PID_StatusTypeDef PID3_Set_Integrator_Limit(PID3 *pid, float min, float max);

/* SET OUTPUT LIMIT */
PID_StatusTypeDef PID3_Set_Output_Limit(PID3 *pid, float min, float max);

/* ADVANCE TIMESTEP FUNCTION */
PID_StatusTypeDef PID3_Update(PID3 *pid, float *set, float *rot, float T);

#endif /* INC_PID3_H_ */
