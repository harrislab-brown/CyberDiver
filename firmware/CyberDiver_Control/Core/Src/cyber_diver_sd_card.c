/*
 * cyber_diver_sd_card.c
 *
 *  Created on: Oct 30, 2024
 *      Author: johnt
 */

#include "cyber_diver_sd_card.h"

static FATFS fs;
static FIL fil;
static UINT bw;

uint32_t CyberDiverSD_CreateDataFile(void)
{
	/*
	 * we first need to determine an appropriate file name for the data file before we create it.
	 * we want the file names to automatically increment so several trials can be performed without taking data off the SD card.
	 * Data files will be named "data###.dat" where ### is the trial number. when making a new file, we want to looks at all the
	 * files in the root directory that match this pattern and get the largest ### that has been written so far. Then the current
	 * file will we numbered ### + 1
	 */

	/* try to mount the file system */
	if (f_mount(&fs, "/", 1) != FR_OK)
		return 0;

	/* get all the file names in the root directory and check if they match the pattern */
	DIR dir;
	FILINFO fno;
	uint8_t trial_number = 0;

	uint32_t nlen = strlen(CD_DATA_FILE_NAME);
	uint32_t elen = strlen(CD_DATA_FILE_EXT);

	(void)f_opendir(&dir, "/");
	do
	{
		(void)f_readdir(&dir, &fno);
		if (fno.fname[0])
		{
			/* we found a file, check the name */
			uint32_t flen = strlen(fno.fname);

			if (!strncmp(CD_DATA_FILE_NAME, fno.fname, nlen) && !strncmp(CD_DATA_FILE_EXT, fno.fname + flen - elen, elen))  /* check name and extension matches */
			{
				char num[8];  /* allow up to 8 digits */
				strncpy(num, fno.fname + nlen, flen - nlen - elen);  /* extract numeric substring (this assumes that the substring is numeric which isn't guaranteed in general but who cares it'll work) */
				num[flen - nlen - elen] = '\0';  /* add terminator */
				if (atoi(num) > trial_number)
				{
					trial_number = atoi(num);  /* keep track of the largest trial number found */
				}
			}
		}
	} while (fno.fname[0]);
	(void)f_closedir(&dir);

	/* now that we know how many trials have been performed, open the new file */
	char file_name[13];  /* unless using long file names, the max length in 13 characters in FATFS */
	strncpy(file_name, CD_DATA_FILE_NAME, nlen);
	uint32_t ilen = snprintf(NULL, 0, "%d", ++trial_number);  /* figure out the length of the trial number string */
	snprintf(file_name + nlen, ilen + 1, "%d", trial_number);  /* write the trial number */
	strncpy(file_name + nlen + ilen, CD_DATA_FILE_EXT, elen);  /* add the extension */
	file_name[nlen + ilen + elen] = '\0';

	/* open a file to start logging */
	return f_open(&fil, file_name, FA_CREATE_ALWAYS|FA_WRITE) == FR_OK;
}


uint32_t CyberDiverSD_WriteData(uint8_t* data, uint32_t size)
{
	return f_write(&fil, data, size, &bw) == FR_OK;

}

uint32_t CyberDiverSD_SaveDataFile(void)
{
	if (f_close(&fil) != FR_OK)  /* close the file */
		return 0;

	if (f_mount(NULL, "/", 1) != FR_OK)  /* dismount the file system */
		return 0;

	return 1;
}

FIL* CyberDiverSD_GetFile(void)
{
	return &fil;
}
