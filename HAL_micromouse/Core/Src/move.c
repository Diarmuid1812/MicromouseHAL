/*
 * move.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Mikolaj Szustakiewicz
 */

#include "move.h"

void motorsInit()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

/*
 *  Funkcje ustawiające kierunek obrotu silnika i prędkośc
 *
 *  Prędkosc ustawiana jest w 0.1% wypelnienia PWM
 *  Kierunek - naprzód: 1, wstecz: -1.
 */
void setMoveR(int8_t movementDir, uint32_t Comp)
{
	switch(movementDir)
	{
		case -1:
			HAL_GPIO_WritePin(DIR_R_GPIO_Port,DIR_R_Pin,1);
			MOTR_SetCompare(Comp);
			break;
		case 1:
			HAL_GPIO_WritePin(DIR_R_GPIO_Port,DIR_R_Pin,0);
			MOTR_SetCompare(Comp);
			break;
		default:
			return;
	}
}

void setMoveL(int8_t movementDir, uint32_t Comp)
{
	switch(movementDir)
	{
		case -1:
			HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, 1);
			MOTL_SetCompare(Comp);
			break;
		case 1:
			HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, 0);
			MOTL_SetCompare(Comp);
			break;
		default:
			return;
	}
}

/*
 * Funkcja wykonuje ruch naprzód o zadaną w milimetrach odległość. Zwraca 1, gdy zakończyła jazdę i 0, gdy nadal jedzie
 * średnica koła - 37.2mm
 * ilość tików na obrót - 900
 * ilość tików na milimetr - 7.70105
 *
 */

uint8_t move_forward(uint32_t distance_mm, uint32_t speed)
{
	int cv = 0;
	encRead();
	if (((rightTotal + leftTotal) / 2) < 7.70105 * distance_mm)
	{
		if (pidChangedFlag)
		{
			pidChangedFlag = 0;
			cv = pidCalc(eps);

			setMoveR(1, speed - cv/2);
			setMoveL(1, speed + cv/2);
		}
		return 0;
	}
	else
	{
		setMoveR(1, 0);
		setMoveL(1, 0);
		encReset();
		return 1;
	}
}

/*
 * Funkcja wykonuje ruch do tyłu o zadaną w milimetrach odległość. Zwraca 1, gdy zakończyła jazdę i 0, gdy nadal jedzie
 * średnica koła - 37.2mm
 * ilość tików na obrót - 900
 * ilość tików na milimetr - 7.70105
 *
 */
uint8_t move_back(uint32_t distance_mm, uint32_t speed)
{
	int cv = 0;
	encRead();
	if (((rightTotal + leftTotal) / 2) > -7.70105 * distance_mm)
	{
		if (pidChangedFlag)
		{
			pidChangedFlag = 0;
			cv = pidCalc(eps);

			setMoveR(-1, speed + cv/2);
			setMoveL(-1, speed - cv/2);
		}
		return 0;
	}
	else
	{
		setMoveR(-1, 0);
		setMoveL(-1, 0);
		encReset();
		return 1;
	}
}

void turn_right()
{
	encRead();

	int16_t rotTmp = rotTotal;

	while(rotTotal > rotTmp-911) { //Wartosc obr do zwrotu o 90 deg
	setMoveR(-1,200);
	setMoveL(1,200);

	//encodersRead();

	HAL_Delay(10);
	}

	setMoveR(1,0);
	setMoveL(1,0);
}

void turn_left()
{
	encRead();

	int16_t rotTmp = rotTotal;

	while(rotTotal < rotTmp+980) { //Wartosc obr do zwrotu o 90 deg
	setMoveR(1,200);
	setMoveL(-1,200);

	encRead();

	HAL_Delay(10);
	}

	setMoveR(1,0);
	setMoveL(1,0);
}
