/*
 * cyber_diver_sd_card.h
 *
 *  Created on: Oct 30, 2024
 *      Author: johnt
 */

#ifndef INC_CYBER_DIVER_SD_CARD_H_
#define INC_CYBER_DIVER_SD_CARD_H_

/*
 * handles writing the experimental data files to the SD card
 */


#include "fatfs.h"
#include <string.h>
#include <stdio.h>


#define CD_DATA_FILE_NAME "DATA"
#define CD_DATA_FILE_EXT ".DAT"

uint32_t CyberDiverSD_CreateDataFile(void);
uint32_t CyberDiverSD_WriteData(uint8_t* data, uint32_t size);
uint32_t CyberDiverSD_SaveDataFile(void);
FIL* CyberDiverSD_GetFile(void);

#endif /* INC_CYBER_DIVER_SD_CARD_H_ */
