/*
 * pid.h
 *
 *  Created on: May 11, 2020
 *      Author: Mikolaj Szustakiewicz
 */

#ifndef INC_PID_H_
#define INC_PID_H_

float pidKp;
float pidKi;
float pidKd;
float ep;      //uchyb poprzedni

float C;    //całka
float U;    //wyjście regulatora
float dt;   //podstawa czasu

void pidInit(float kp_param, float ki_param, float kd_param);

void pidReset();

float pidCalc(float eps);

#endif /* INC_PID_H_ */
