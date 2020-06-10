/*
 * mapping.h
 *
 *  Created on: 23 Gibl 2020
 *      Author: Mikołaj Szustakiewicz
 */

#ifndef INC_MAPPING_H_
#define INC_MAPPING_H_

#include "stm32f4xx_hal.h"

#define LABIRYNTH_SIZE 4


/**mapa -- reprezentacja:
 * 0000 W_wall_bit S_wall_bit E_wall_bit N_wall_bit
 * 1 index - oś X
 * 2 index - oś Y
 * Poczatek indexowania
 * - lewy dolny rog
 */

//uint8_t map[LABIRYNTH_SIZE][LABIRYNTH_SIZE];

typedef struct{
	uint8_t visited;
	uint8_t walls;
	uint8_t distance;
} field;


field map[LABIRYNTH_SIZE][LABIRYNTH_SIZE];

/* directions to access map
 * Usage: isWall = (map[X][Y]&(1u<<Dir))
 */
typedef enum{N,E,S,W} DirType;

typedef struct{
	uint8_t pX;
	uint8_t pY;
}QNode;

typedef struct{
	uint8_t size;
	uint8_t endInd;
	QNode content[LABIRYNTH_SIZE*LABIRYNTH_SIZE];
}FieldQue;

uint8_t isQEmpty(FieldQue * Q);

uint8_t isQFull(FieldQue * Q);

void initQue(FieldQue * Q);

void deque(FieldQue * Q, uint8_t* pX, uint8_t* pY);
void enque(FieldQue * Q, uint8_t pX, uint8_t pY);

void mapInit();

#endif /* INC_MAPPING_H_ */
