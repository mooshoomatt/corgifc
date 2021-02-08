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
PID_StatusTypeDef PID3_Init(PID3 *pid, float *p, float *i, float*d)
{
	return PID_OK;
}

/* SET INTEGRATOR LIMIT */
PID_StatusTypeDef PID3_Set_Integrator_Limit(PID3 *pid, float min, float max)
{
	return PID_OK;
}

/* SET OUTPUT LIMIT */
PID_StatusTypeDef PID3_Set_Output_Limit(PID3 *pid, float min, float max)
{
	return PID_OK;
}

/* ADVANCE TIMESTEP FUNCTION */
PID_StatusTypeDef PID3_Update(PID3 *pid, float T)
{
	return PID_OK;
}
