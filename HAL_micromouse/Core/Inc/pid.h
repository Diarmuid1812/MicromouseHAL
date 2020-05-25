/*
 * pid.h
 *
 *  Created on: May 11, 2020
 *      Author: Mikolaj Szustakiewicz
 */

#ifndef INC_PID_H_
#define INC_PID_H_
typedef struct
{
	//nastawy regulatora
	float pidKp;
	float pidKi;
	float pidKd;

	//uchyb poprzedni
	float ep;

	//całka
	float C;
	//wyjście regulatora
	float U;
	//podstawa czasu
	float dt;
}PIDtype_f;

void pidInit(PIDtype_f * pid, float kp_param, float ki_param, float kd_param, float dt_param);

void pidReset(PIDtype_f * pid);
float pidCalc(PIDtype_f * pid, float eps);

#endif /* INC_PID_H_ */
