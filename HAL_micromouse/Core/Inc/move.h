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

#define MOTR_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Compare)
#define MOTL_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Compare)

extern volatile uint8_t pidChangedFlag;
extern volatile float eps;

void motorsInit();

void setMoveR(int8_t movementDir, uint32_t Comp);

void setMoveL(int8_t movementDir, uint32_t Comp);

uint8_t move_forward(uint32_t distance_mm, uint32_t speed);

uint8_t move_back(uint32_t distance_mm, uint32_t speed);

void turn_right();
void turn_left();


#endif /* INC_MOVE_H_ */
