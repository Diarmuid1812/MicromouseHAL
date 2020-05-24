/*
 * pid.c
 *
 *  Created on: 10 maj 2020
 *      Author: Mikolaj Szustakiewicz
 */

#include "pid.h"

void pidInit(float kp_param, float ki_param, float kd_param)
{
	pidKp = kp_param;
	pidKi = ki_param;
	pidKd = kd_param;
	ep = 0;
	C = 0;
	U = 0;
}

void pidReset()
{
	ep = 0;
	C = 0;
	U = 0;
}

float pidCalc(float eps)
{
	C+=((ep + eps)/2)*dt;
	U=pidKp*(ep + pidKi*C + pidKd*(ep-eps)/dt);
	ep = eps;

	return U;
}
