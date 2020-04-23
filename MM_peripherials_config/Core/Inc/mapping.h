/*
 * mapping.h
 *
 *  Created on: 23 Gibl 2020
 *      Author: szust
 */

#ifndef INC_MAPPING_H_
#define INC_MAPPING_H_

#include "stm32f4xx_hal.h"


/**mapa -- reprezentacja:
 * 0000 W_wall_bit S_wall_bit E_wall_bit N_wall_bit
 * 1 index - oś X
 * 2 index - oś Y
 * Poczatek indexowania
 * - lewy dolny rog
 */

uint8_t map[16][16];
/* directions to access map
 * Usage: isWall = (map[X][Y]&(1u<<Dir))
 */
typedef enum{N,E,S,W} DirType;


#endif /* INC_MAPPING_H_ */
