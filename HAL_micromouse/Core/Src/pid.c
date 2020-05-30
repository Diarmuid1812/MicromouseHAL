/*
 * pid.c
 *
 *  Created on: 10 maj 2020
 *      Author: Mikolaj Szustakiewicz
 */

#include "pid.h"

void pidInit(PIDtype_f * pid, float kp_param, float ki_param, float kd_param, float dt_param, float C_windup_param)
{
	pid->pidKp = kp_param;
	pid->pidKi = ki_param;
	pid->pidKd = kd_param;
	pid->C_windup = C_windup_param;

	pid->ep = 0;
	pid->C = 0;
	pid->U = 0;
	pid->dt = dt_param;
}

void pidReset(PIDtype_f * pid)
{
	pid->ep = 0;
	pid->C = 0;
	pid->U = 0;
}


float pidCalc(PIDtype_f * pid, float eps)
{
	pid->C+=((pid->ep + eps)/2)*pid->dt;   //obliczenie całki

	if(pid->C > pid->C_windup)    //windup członu całkującego
	{
		pid->C = pid->C_windup;
	}
	if(pid->C < -pid->C_windup)
	{
		pid->C = -pid->C_windup;
	}

	pid->U=(pid->pidKp*pid->ep)  +  (pid->pidKi*pid->C)  +  (pid->pidKd*(pid->ep-eps)/pid->dt);   //suma członów

	pid->ep = eps; //nadpisanie starego uchybu nowym

	return pid->U;
}
