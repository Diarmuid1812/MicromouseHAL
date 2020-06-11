/*
 * FloodFill.c
 *
 *  Created on: 14 maj 2020
 *      Author: Mikołaj Szustakiewicz
 */

#include "mapping.h"
#include "VL53L0x.h"


void checkWalls(uint8_t posX, uint8_t posY, DirType direction)
{
	DirType dirL = direction == N ? W : direction-1;
	DirType dirR = (direction+1)%4;
	if(isWall(&ToF_F))
	{
		map[posX][posY].walls |= 1u<<direction;
		if(direction==N&&posY<LABIRYNTH_SIZE-1)
		{
			map[posX][posY+1].walls|=1u<<S;
		}
		if(direction==E&&posX<LABIRYNTH_SIZE-1)
		{
			map[posX+1][posY].walls|=1u<<W;
		}
		if(direction==S&&posY>0)
		{
			map[posX][posY-1].walls|=1u<<N;
		}
		if(direction==W&&posX>0)
		{
			map[posX-1][posY].walls|=1u<<E;
		}
	}
	if(isWall(&ToF_R))
	{

		map[posX][posY].walls |= 1u<<dirR;
		if(dirR==N&&posY<LABIRYNTH_SIZE-1)
		{
			map[posX][posY+1].walls|=1u<<S;
		}
		if(dirR==E&&posX<LABIRYNTH_SIZE-1)
		{
			map[posX+1][posY].walls|=1u<<W;
		}
		if(dirR==S&&posY>0)
		{
			map[posX][posY-1].walls|=1u<<N;
		}
		if(dirR==W&&posX>0)
		{
			map[posX-1][posY].walls|=1u<<E;
		}
	}
	if(isWall(&ToF_L))
	{
		map[posX][posY].walls |= 1u<<dirL;
		if(dirL==N&&posY<LABIRYNTH_SIZE-1)
		{
			map[posX][posY+1].walls|=1u<<S;
		}
		if(dirL==E&&posX<LABIRYNTH_SIZE-1)
		{
			map[posX+1][posY].walls|=1u<<W;
		}
		if(dirL==S&&posY>0)
		{
			map[posX][posY-1].walls|=1u<<N;
		}
		if(dirL==W&&posX>0)
		{
			map[posX-1][posY].walls|=1u<<E;
		}
	}

}

DirType floodFill(uint8_t posX, uint8_t posY, DirType dir)
{
	uint8_t newX=0;
	uint8_t newY=0;
	uint8_t minDist=255;
	uint8_t checkDist=255;
	DirType nextField=N;

	FieldQue Q;
	initQue(&Q);

	enque(&Q, LABIRYNTH_SIZE-1, LABIRYNTH_SIZE-1);

	if(posX==0&&posY==0)map[posX][posY].walls |= 1u<<S;
	checkWalls(posX, posY, dir);

	while(!isQEmpty(&Q))
	{
		deque(&Q, &newX, &newY);

		if(!map[newX][newY].visited)
		{
			map[newX][newY].visited = 1;
			if(newY<LABIRYNTH_SIZE-1) if(!(map[newX][newY].walls&(1u<<N))) enque(&Q, newX , newY+1);
			if(newX<LABIRYNTH_SIZE-1) if(!(map[newX][newY].walls&(1u<<E)))enque(&Q, newX+1, newY);
			if(newY>0) if(!(map[newX][newY].walls&(1u<<S))) enque(&Q, newX , newY-1);
			if(newX>0) if(!(map[newX][newY].walls&(1u<<W))) enque(&Q, newX-1, newY);
		}
		if((newX==LABIRYNTH_SIZE-1&&newY==LABIRYNTH_SIZE-1))map[newX][newY].distance = 0;
		//if((newX==8&&newY==8)||(newX==8&&newY==9)||(newX==9&&newY==8)||(newX==9&&newY==9))
			//map[newX][newY].distance = 0;
		else
		{
			minDist=255;
			if(newY<LABIRYNTH_SIZE-1 && !(map[newX][newY].walls&(1u<<N)))
			{
				checkDist = map[newX][newY+1].distance;
				if(checkDist < minDist) minDist = checkDist;
			}
			if(newX<LABIRYNTH_SIZE-1 && !(map[newX][newY].walls&(1u<<E)))
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

	minDist=255;
	if(newY<LABIRYNTH_SIZE-1 && !(map[posX][posY].walls&(1u<<N)))
	{
		checkDist = map[posX][posY+1].distance;
		if(checkDist < minDist) {minDist = checkDist; nextField=N;}
	}
	if(posX<LABIRYNTH_SIZE-1 && !(map[posX][posY].walls&(1u<<E)))
	{
		checkDist = map[posX+1][posY].distance;
		if(checkDist < minDist) {minDist = checkDist;nextField=E;}
	}
	if(posY>0 && !(map[posX][posY].walls&(1u<<S)))
	{
		checkDist = map[posX][posY-1].distance;
		if(checkDist < minDist) {minDist = checkDist; nextField=S;}
	}
	if(posX>0 && !(map[posX][posY].walls&(1u<<W)))
	{
		checkDist = map[posX-1][posY].distance;
		if(checkDist < minDist) {minDist = checkDist; nextField=W;}
	}

	for(uint8_t j=0;j<LABIRYNTH_SIZE;++j)
		for(uint8_t i=0;i<LABIRYNTH_SIZE;++i)
			map[i][j].visited=0;
	return nextField;
}

