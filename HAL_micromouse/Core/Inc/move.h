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

//parametry regulatora PID
#define PID_PARAM_KP        5         /* Proportional */
#define PID_PARAM_KI        8         /* Integral */
#define PID_PARAM_KD        0.001     /* Derivative */
#define PID_DT              0.001     /*Time loop duration*/
//PID dla poruszania na wprost
PIDtype_f pidSt;
/************************************/

/***********************************/


#define MOTR_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Compare)
#define MOTL_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Compare)

extern volatile uint8_t pidChangedFlag;
extern volatile float eps;

void motorsInit();

void setMoveR(int8_t movementDir, uint32_t Comp);

void setMoveL(int8_t movementDir, uint32_t Comp);

uint8_t move_forward(uint32_t distance_mm, uint32_t speed, PIDtype_f * pid);

uint8_t move_back(uint32_t distance_mm, uint32_t speed, PIDtype_f * pid);

void turn_right();
void turn_left();



#endif /* INC_MOVE_H_ */
