/*
 * encoders.c
 *
 *  Created on: Apr 23, 2020
 *      Author: ---
 */

#include "encoders.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;


void encRead(void)
{


  oldLeftEncoder = leftEncoder;
  leftEncoder = TIM2->CNT; // przepisanie wartosci z rejestru timera 2 do zmiennej
  oldRightEncoder = rightEncoder;
  rightEncoder = -TIM4->CNT; // przepisanie wartosci z rejestru timera 4 do zmiennej
  leftCount = leftEncoder - oldLeftEncoder;
  rightCount = rightEncoder - oldRightEncoder;
  fwdCount = leftCount + rightCount;
  rotCount = - (leftCount - rightCount);
  fwdTotal += fwdCount;
  rotTotal += rotCount;
  leftTotal += leftCount;
  rightTotal += rightCount;


}

void encReset (void)
{
  __disable_irq();
  oldLeftEncoder = 0;
  oldRightEncoder = 0;
  leftTotal = 0;
  rightTotal = 0;
  fwdTotal = 0;
  rotTotal = 0;
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  encRead();
  __enable_irq();
}
