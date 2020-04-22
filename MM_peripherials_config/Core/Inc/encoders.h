/*
 * encoders.h
 *
 *  Created on: Apr 23, 2020
 *      Author: szust
 */

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#include "stm32f4xx_hal.h"

volatile int16_t leftCount;
volatile int16_t rightCount;
volatile int16_t fwdCount;
volatile int16_t rotCount;
//distances
volatile int32_t leftTotal;
volatile int32_t rightTotal;
volatile int32_t fwdTotal;
volatile int32_t rotTotal;
// local variables
static volatile int16_t oldLeftEncoder;
static volatile int16_t oldRightEncoder;
static volatile int16_t leftEncoder;
static volatile int16_t rightEncoder;
static volatile int16_t encoderSum;
static volatile int16_t encoderDiff;

void encRead(void);
void encReset (void);

#endif /* INC_ENCODERS_H_ */
