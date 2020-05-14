/*
 * FloodFill.c
 *
 *  Created on: 14 maj 2020
 *      Author: Mikołaj Szustakiewicz
 */

#include "mapping.h"

void floodFill(uint8_t posX, uint8_t posY)
{
	uint8_t newX=0;
	uint8_t newY=0;
	uint8_t minDist=0;
	uint8_t checkDist=0;

	FieldQue Q;
	initQue(&Q);

	enque(&Q, posX, posY);
	while(!isQEmpty(&Q))
	{
		deque(&Q, &newX, &newY);

		if(!map[newX][newY].visited)
		{
			map[newX][newY].visited = 1;
			if(newY<255) if(!(map[newX][newY].walls&(1u<<N))) enque(&Q, newX , newY+1);
			if(newX<255) if(!(map[newX][newY].walls&(1u<<E)))enque(&Q, newX+1, newY);
			if(newY>0) if(!(map[newX][newY].walls&(1u<<S))) enque(&Q, newX , newY-1);
			if(newX>0) if(!(map[newX][newY].walls&(1u<<W))) enque(&Q, newX-1, newY);
		}
		if((newX==8&&newY==8)||(newX==8&&newY==9)||(newX==9&&newY==8)||(newX==9&&newY==9))
			map[newX][newY].distance = 0;
		else
		{
			minDist=255;
			if(newY<255 && !(map[newX][newY].walls&(1u<<N)))
			{
				checkDist = map[newX][newY+1].distance;
				if(checkDist < minDist) minDist = checkDist;
			}
			if(newX<255 && !(map[newX][newY].walls&(1u<<E)))
			{
				checkDist = map[newX+1][newY].distance;
				if(checkDist < minDist) minDist = checkDist;
			}
			if(newY>0 && !(map[newX][newY].walls&(1u<<S)))
			{
				checkDist = map[newX][newY-1].distance;
				if(checkDist < minDist) minDist = checkDist;
			}
			if(newX>0 && !(map[newX][newY].walls&(1u<<W)))
			{
				checkDist = map[newX-1][newY].distance;
				if(checkDist < minDist) minDist = checkDist;
			}
			map[newX][newY].distance = minDist + 1;

		}
	}
}

