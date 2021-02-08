/*
 * PID1.c
 *
 * SINGLE AXIS PID CONTROLLER
 *
 *  Created on: Feb 7, 2021
 *      Author: Matthew Morawiec
 */

#include "PID1.h"

/* INITIALIZATION FUNCTION */
PID_StatusTypeDef PID1_Init(PID1 *pid, float p, float i, float d)
{
	/* CLEAR ALL VARIABLES */
	pid->set        = 0.0;
	pid->output     = 0.0;
	pid->integral   = 0.0;
	pid->prevError  = 0.0;
	pid->derivative = 0.0;
	pid->prevMeas   = 0.0;

	/* SET GAINS */
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;

	/* SET CLAMPS */
	pid->intLimMax = PID_MAXINT;
	pid->intLimMin = PID_MININT;
	pid->outLimMax = PID_MAXOUT;
	pid->outLimMin = PID_MINOUT;

	if ((p <= 0) & (i <= 0) & (d <= 0)) { return PID_ERROR; }

	return PID_OK;
}

/* SET FILTER TIME CONSTANT */
PID_StatusTypeDef PID1_Set_Tau(PID1 *pid, float t)
{
	if (t < 0.0) { return PID_ERROR; }

	pid->tau = t;

	return PID_OK;
}

/* SET INTEGRATOR LIMIT */
PID_StatusTypeDef PID1_Set_Integrator_Limit(PID1 *pid, float min, float max)
{
	if ((min >= 0.0) | (max <= 0.0)) { return PID_ERROR; }

	pid->intLimMax = max;
	pid->intLimMin = min;

	return PID_OK;
}

/* SET OUTPUT LIMIT */
PID_StatusTypeDef PID1_Set_Output_Limit(PID1 *pid, float min, float max)
{
	if ((min >= 0.0) | (max <= 0.0)) { return PID_ERROR; }

	pid->outLimMax = max;
	pid->outLimMin = min;

	return PID_OK;
}

/* ADVANCE TIMESTEP FUNCTION */
PID_StatusTypeDef PID1_Update(PID1 *pid, float T)
{
	// CALCULATE ERROR
	pid->error = pid->set - pid->meas;

	// CALCULATE PROPORTIONAL TERM
	pid->proportional = pid->Kp * pid->error;

	// CALCULATE INTEGRAL TERM AND CLAMP
	pid->integral = pid->integral + 0.5*(pid->Ki)*T*(pid->error + pid->prevError);
	pid->integral = (pid->integral > pid->intLimMax) ? pid->intLimMax : pid->integral;
	pid->integral = (pid->integral < pid->intLimMin) ? pid->intLimMin : pid->integral;

	// CALCULATE DERIVATIVE TERM (ON MEASUREMENT)
	if (T == 0) { return PID_ERROR; }
	pid->derivative = -(2.0*(pid->Kd)*(pid->meas - pid->prevMeas) + (2.0*pid->tau - T)*pid->derivative)/(2.0*pid->tau + T);

	// CALCULATE OUTPUT AND CLAMP
	pid->output = pid->proportional + pid->integral + pid->derivative;
	pid->output = (pid->output > pid->outLimMax) ? pid->outLimMax : pid->output;
	pid->output = (pid->output < pid->outLimMin) ? pid->outLimMin : pid->output;

	// UPDATE TEMPVARS
	pid->prevError = pid->error;
	pid->prevMeas  = pid->meas;

	return PID_OK;
}
