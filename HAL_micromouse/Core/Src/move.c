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
			cv = pidCalc(&pidSt, eps);

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
			cv = pidCalc(&pidSt,eps);

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


//skret w prawo - wartość ujemna, skręt w lewo - wartość dodatnia. Jak kąty na układnie współrzędnych.

uint8_t turn_right(float currentAngle, float destAngle, uint32_t speed)
{
	if (currentAngle > destAngle + 1000)
	{
		setMoveR(-1, speed);    //100 optymalna wartość dla prędkości obrotu
		setMoveL(1, speed);

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

uint8_t turn_left(float currentAngle, float destAngle, uint32_t speed)
{
	if (currentAngle < destAngle - 1000)
	{
		setMoveR(1, speed);    //100 optymalna wartość dla prędkości obrotu
		setMoveL(-1, speed);

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

uint8_t turn_pid(float currentAngle, float destAngle)
{
	if (currentAngle > destAngle + 100 || currentAngle < destAngle - 100) //skręt w prawo
	{
		counter = 0;
		if (pidChangedFlag)
		{
			pidChangedFlag = 0;

			int32_t cv = pidCalc(&pidAngle, destAngle - currentAngle);

			if (cv > 400) //400 - maksymalna prędkość obrotu silników przy obrocie
				cv = 400;
			if (cv < -400)
				cv = -400;

			if (cv < 0)
			{
				setMoveR(-1, 35 - cv / 2);
				setMoveL(1, 35 - cv / 2);
			}
			else if (cv > 0)
			{
				setMoveR(1, 35 + cv / 2);
				setMoveL(-1, 35 + cv / 2);
			}
		}

		return 0;
	}

	else
	{
		counter++;
		if(counter == 2)
		{
			setMoveR(-1, 0);
			setMoveL(-1, 0);
			encReset();
			return 1;
		}
		return 0;
	}
}
