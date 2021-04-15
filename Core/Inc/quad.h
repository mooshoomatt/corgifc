/*
 * quad.h
 *
 *  Created on: Feb 21, 2021
 *      Author: matth
 */

#ifndef INC_QUAD_H_
#define INC_QUAD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <math.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "BMI088.h"
#include "PID3.h"
#include "oneshot125.h"

#define GYRO_RATE 2000.0  // Gyroscope output scale (deg/second)
#define IC_SCALE  1000.0  // IC Pulse width scale

#define X_DEFAULT_RATE 5.0  // DEFAULT ROLL RATE  (rad/s)
#define Y_DEFAULT_RATE 5.0  // DEFAULT PITCH RATE (rad/s)
#define Z_DEFAULT_RATE -3.0 // DEFAULT YAW RATE   (rad/s)

#define ARM_CHANNEL 4      // ARRAY INDEX FOR ARM SWITCH

typedef struct {

	I2C_HandleTypeDef *hi2c;  // I2C Handle for gyro
	TIM_HandleTypeDef *htim;  // Timer Handle for time measurement
	PWMCAPTURE *IC;           // Input Capture Handle
	ONESHOT125 *OS;           // OneShot125 Output Handle
	PID3 *PID;                // PID Controller Handle

	uint16_t tprev, telapsed; // Timekeeping Variables

	float RATES[3];           // CONTROL RATES
	float rot[3];             // MEASURED ROTATION
	float set[3];             // SETPOINT

	uint8_t ARM_STATUS;       // ARM STATUS
	uint8_t TEST_MODE;        // THROTTLE PASS THROUGH MODE

	uint8_t gyro_buf[6];      // GYROSCOPE BYTE BUFFER
	char tx_buf[256];         // TX BUFFER

	float  gyro_rate[3];      // GYROSCOPE RATE BUFFER
	float  stick_rate[3];     // CONTROL RATE BUFFER

	int16_t temp_i;			  // GYRO CONVERSION TEMPVAR

} QUAD;

/* INITIALIZE QUAD ROTOR STRUCT */
void QUAD_Init(QUAD *quad);

/* CLEAR STATE FUNCTION */
void QUAD_Clear(QUAD *quad);

/* UPDATE PROCEDURE */
void QUAD_UPDATE(QUAD *quad);

/* SEND ORIENTATION OVER SERIAL */
void QUAD_SEND_ORIENTATION(QUAD *quad);

#endif /* INC_QUAD_H_ */
