/*
 * move.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Mikolaj Szustakiewicz
 */


#include "main.h"
#include "move.h"
#include "encoders.h"

extern


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
			HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, 1);
			MOTL_SetCompare(Comp);
			break;
		default:
			return;
	}
}

void move_fwd()
{//TODO dodac PID
	int16_t fwdTmp = fwdTotal;
	/* ??? todo: Do czego to było ???*/
	while(fwdTotal < fwdTmp+2000) { //Wartosc obr do zwrotu o 90 deg
		setMoveR(1,300);
		setMoveL(1,300);

		encRead();

		HAL_Delay(10);
		}

		setMoveR(1,0);
		setMoveL(1,0);

}

void move_back()
{//TODO dodac PID
	int16_t fwdTmp = fwdTotal;
	/* ??? todo: Do czego to było ???*/
	while(fwdTotal < fwdTmp+2000) { //Wartosc obr do zwrotu o 90 deg
		setMoveR(-1,300);
		setMoveL(-1,300);

		encRead();

		HAL_Delay(10);
		}

		setMoveR(1,0);
		setMoveL(1,0);

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
