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
static float ep = 0;      //uchyb poprzedni

static float C = 0;       //całka
static float U = 0;       //wyjście regulatora
const float dt = 0.001;   //podstawa czasu

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
