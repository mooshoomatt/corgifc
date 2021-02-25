/*
 * PID3.c
 *
 * 3-AXIS PID CONTROLLER
 *
 *  Created on: Feb 7, 2021
 *      Author: Matthew Morawiec
 */

#include "PID3.h"

/* INITIALIZATION FUNCTION */
PID_StatusTypeDef PID3_Init(PID3 *pid, const float *p, const float *i, const float*d)
{
	/* SET PID GAINS */
	pid->Kp[0] = p[0]; pid->Kp[1] = p[1]; pid->Kp[2] = p[2];

	pid->Ki[0] = i[0]; pid->Ki[1] = i[1]; pid->Ki[2] = i[2];

	pid->Kd[0] = d[0]; pid->Kd[1] = d[1]; pid->Kd[2] = d[2];

	/* CLEAR IMPORTANT VARIABLES */
	pid->out[0] = 0.0; pid->out[1] = 0.0; pid->out[2] = 0.0;

	pid->integral[0] = 0.0; pid->integral[1] = 0.0; pid->integral[2] = 0.0;

	pid->prevError[0] = 0.0; pid->prevError[1] = 0.0; pid->prevError[2] = 0.0;

	pid->prevRot[0] = 0.0; pid->prevRot[1] = 0.0; pid->prevRot[2] = 0.0;

	/* SET CLAMPS */
	pid->intLimMax = PID_MAXINT;
	pid->intLimMin = PID_MININT;
	pid->outLimMax = PID_MAXOUT;
	pid->outLimMin = PID_MINOUT;

	return PID_OK;
}

/* CLEAR STATE FUNCTION */
PID_StatusTypeDef PID3_Clear(PID3 *pid)
{
	/* CLEAR IMPORTANT VARIABLES */
	pid->out[0] = 0.0; pid->out[1] = 0.0; pid->out[2] = 0.0;

	pid->integral[0] = 0.0; pid->integral[1] = 0.0; pid->integral[2] = 0.0;

	pid->prevError[0] = 0.0; pid->prevError[1] = 0.0; pid->prevError[2] = 0.0;

	pid->prevRot[0] = 0.0; pid->prevRot[1] = 0.0; pid->prevRot[2] = 0.0;

	return PID_OK;
}

/* SET FILTER TIME CONSTANT */
PID_StatusTypeDef PID3_Set_Tau(PID3 *pid, float t)
{
	if (t < 0.0) { return PID_ERROR; }

	pid->tau = t;

	return PID_OK;
}

/* SET INTEGRATOR LIMIT */
PID_StatusTypeDef PID3_Set_Integrator_Limit(PID3 *pid, float min, float max)
{
	if ((min >= 0.0) | (max <= 0.0)) { return PID_ERROR; }

	pid->intLimMax = max;
	pid->intLimMin = min;

	return PID_OK;
}


/* SET OUTPUT LIMIT */
PID_StatusTypeDef PID3_Set_Output_Limit(PID3 *pid, float min, float max)
{
	if ((min >= 0.0) | (max <= 0.0)) { return PID_ERROR; }

	pid->outLimMax = max;
	pid->outLimMin = min;

	return PID_OK;
}

/* ADVANCE TIMESTEP FUNCTION */
PID_StatusTypeDef PID3_Update(PID3 *pid, float *set, float *rot, float T)
{
	// FOR EACH AXIS
	for (int i = 0; i < 3; i++)
	{
		// CALCULATE ERROR
		pid->error[i] = set[i] - rot[i];

		// CALCULATE PROPORTIONAL TERM
		pid->proportional[i] = pid->Kp[i] * pid->error[i];

		// CALCULATE INTEGRAL TERM AND CLAMP
		pid->integral[i] += 0.5*(pid->Ki[i])*T*(pid->error[i] + pid->prevError[i]);
		pid->integral[i] = fmax(pid->integral[i], pid->intLimMin);
	    pid->integral[i] = fmin(pid->integral[i], pid->intLimMax);

		// CALCULATE DERIVATIVE TERM (ON MEASUREMENT)
		if (T == 0) { return PID_ERROR; }
		pid->derivative[i] = -(2.0*(pid->Kd[i])*(rot[i] - pid->prevRot[i]) + (2.0*pid->tau - T)*pid->derivative[i])/(2.0*pid->tau + T);

		// CALCULATE OUTPUT AND CLAMP
		pid->out[i] = pid->proportional[i] + pid->integral[i] + pid->derivative[i];
		pid->out[i] = fmax(pid->out[i], pid->outLimMin);
		pid->out[i] = fmin(pid->out[i], pid->outLimMax);

		// OLD CLAMPING METHOD
		//pid->out[i] = (pid->out[i] > pid->outLimMax) ? pid->outLimMax : pid->out[i];
		//pid->out[i] = (pid->out[i] < pid->outLimMin) ? pid->outLimMin : pid->out[i];

		// UPDATE TEMPVARS
		pid->prevError[i] = pid->error[i];
		pid->prevRot[i]   = rot[i];
	}

	return PID_OK;
}
