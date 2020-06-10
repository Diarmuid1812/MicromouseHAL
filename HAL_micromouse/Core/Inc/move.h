/*
 * move.h
 *
 *  Created on: Apr 22, 2020
 *      Author: Mikołaj Szustakiewicz
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include "main.h"
#include "encoders.h"
#include "tim.h"
#include "pid.h"

//Inicjalizacja PID

//parametry regulatora PID do jazdy na wprost
#define PID_ST_PARAM_KP        5         /* Proportional */
#define PID_ST_PARAM_KI        40        /* Integral */
#define PID_ST_PARAM_KD        0.005     /* Derivative */
#define PID_ST_WINDUP          10000     /* Windup integratora */

//parametry regulatora PID do skrętów
#define PID_ANGLE_PARAM_KP     0.020         /* Proportional */
#define PID_ANGLE_PARAM_KI     1.200         /* Integral */
#define PID_ANGLE_PARAM_KD     0.000         /* Derivative */
#define PID_ANGLE_WINDUP       70         /* Windup integratora */

#define PID_DT              0.001     /*Time loop duration*/

#define MOTR_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Compare)
#define MOTL_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Compare)

/***********************************/

//PID dla poruszania na wprost
PIDtype_f pidSt;

//PID do obrotu w osi yaw
PIDtype_f pidAngle;

//licznik zapobiegający wykryciu zerowego uchybu podczas przechodzenia przez zero - wykorzystany w funkcji turn_pid()
uint8_t counter;

/***********************************/




extern volatile uint8_t pidChangedFlag;
extern volatile float eps;

extern float rotx;
extern float gx;

void motorsInit();

void setMoveR(int8_t movementDir, uint32_t Comp);

void setMoveL(int8_t movementDir, uint32_t Comp);

uint8_t move_forward(uint32_t distance_mm, uint32_t speed);

uint8_t move_back(uint32_t distance_mm, uint32_t speed);

uint8_t turn_right(float currentAngle, float destAngle, uint32_t speed);

uint8_t turn_left(float currentAngle, float destAngle, uint32_t speed);

uint8_t turn_pid(float currentAngle, float destAngle);

#endif /* INC_MOVE_H_ */
