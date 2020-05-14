/*
 * mapping.c
 *
 *  Created on: 14 maj 2020
 *      Author: Mikołaj Szustakiewicz
 */

#include "mapping.h"

uint8_t isQEmpty(FieldQue * Q)
{
	return (Q->size==0&&Q->endInd==0);
}

uint8_t isQFull(FieldQue * Q)
{
	return Q->endInd >= 255;
}

void initQue(FieldQue * Q)
{
	Q->size = 0;
	Q->endInd = 0;
}
void deque(FieldQue * Q, uint8_t* pX, uint8_t* pY)
{
		if(isQEmpty(Q)){
			*pX = 90;
			*pY = 90;
			return;
		}
		*pX=Q->content[Q->endInd].pX;
		*pY=Q->content[Q->endInd].pY;
		--Q->size;
		if(Q->size) --Q->endInd;
		return;
}

void enque(FieldQue * Q, uint8_t pX, uint8_t pY)
{
	if(isQFull(Q))
		return;
	if(Q->size)++Q->endInd;
	++Q->size;
	Q->content[Q->endInd].pX=pX;
	Q->content[Q->endInd].pY=pY;
	return;
}

void mapInit()
{
	for(uint8_t px=0; px<16; ++px)
	{
		for(uint8_t py=0; py<16; ++py)
		{
			map[px][py].distance=255;
			map[px][py].visited=0;
			map[px][py].walls=0;
		}
	}
}
