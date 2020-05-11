/*
 * pid.c
 *
 *  Created on: May 11, 2020
 *      Author: Diarmuid
 */


/*
 * pid.c
 *
 *  Created on: 10 maj 2020
 *      Author: Mikolaj Szustakiewicz
 */

#include "pid.h"

static float pidKp = 0;
static float pidKi = 0;
static float pidKd = 0;
static float ep = 0;

static float C = 0;
static float U = 0;
const float dt = 0.5;

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
