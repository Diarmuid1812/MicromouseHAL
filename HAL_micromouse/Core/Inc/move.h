/*
 * move.h
 *
 *  Created on: Apr 22, 2020
 *      Author: szust
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#define MOTR_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Compare)
#define MOTL_SetCompare(Compare)	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Compare)

void motorsInit();

void setMoveR(int8_t movementDir, uint32_t Comp);

void setMoveL(int8_t movementDir, uint32_t Comp);

void move_fwd(void);

void move_back(void);

void turn_right();
void turn_left();


#endif /* INC_MOVE_H_ */
