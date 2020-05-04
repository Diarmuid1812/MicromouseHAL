/*
 * SoftwareCRC.c
 *
 *  Created on: May 3, 2020
 *      Author: Wojtek
 */
#include "SoftwareCRC.h"
#include "stm32f4xx_hal.h"

uint32_t crc32(const void *buf, size_t size)
{
	const uint8_t *p = buf;
	uint32_t crc;

	crc = ~0U;
	while (size--)
	crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
	return crc ^ ~0U;
}

void makeFrame(
		char * frameString,         //frame string to tablica char[50]
		int16_t left_encoder,
		int16_t right_encoder,
		int16_t ToF_L,
		int16_t ToF_FL,
		int16_t ToF_F,
		int16_t ToF_FR,
		int16_t ToF_R)
{

	//sprintf jest dwa razy bo musi nadpisać starą zawartość crc

	sprintf(frameString, "X_%05d_%05d_%04d_%04d_%04d_%04d_%04d_%010lu",
			left_encoder,
			right_encoder,
			ToF_L,
			ToF_FL,
			ToF_F,
			ToF_FR,
			ToF_R,
			crc32(frameString,38));
	sprintf(frameString, "X_%05d_%05d_%04d_%04d_%04d_%04d_%04d_%010lu",
			left_encoder,
			right_encoder,
			ToF_L,
			ToF_FL,
			ToF_F,
			ToF_FR,
			ToF_R,
			crc32(frameString,38));
}
