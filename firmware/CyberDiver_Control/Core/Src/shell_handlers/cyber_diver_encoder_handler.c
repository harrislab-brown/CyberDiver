/*
 * cyber_diver_encoder_handler.c
 *
 *  Created on: Oct 28, 2024
 *      Author: johnt
 */


/*
 * Commands dealing with the nose position encoder
 *
 * >>encoder home
 *
 * >>encoder set offset [offset:float]
 *
 * >>encoder get offset
 *
 * >>encoder get counts
 *
 * >>encoder get position
 *
 */

/*
 * This command handler is a little different because the homing needs to be called at a higher
 * level in the program to make sure we can also disable the controller while homing.
 */


#include "shell_handlers/cyber_diver_encoder_handler.h"


uint32_t CyberDiverEncoderCMD_Set(CyberDiverEncoder* encoder, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "offset"))
		{
			float val;
			if (CyberDiverShell_GetNextFloat(shell, &val))
			{
				CyberDiverEncoder_SetOffset(encoder, val);
				CyberDiverShell_PutOutputString(shell, "ack");
				CyberDiverShell_PutOutputDelimiter(shell);
				return 1;
			}
		}
	}

	return 0;
}


uint32_t CyberDiverEncoderCMD_Get(CyberDiverEncoder* encoder, CyberDiverShell* shell)
{
	char str[CD_SHELL_MAX_TOKEN_LEN];
	if (CyberDiverShell_GetNextString(shell, str, CD_SHELL_MAX_TOKEN_LEN))
	{
		if (!strcmp(str, "offset"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverEncoder_GetOffset(encoder));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "counts"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputInt(shell, CyberDiverEncoder_GetCounts(encoder));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
		else if (!strcmp(str, "position"))
		{
			CyberDiverShell_PutOutputString(shell, "ack");
			CyberDiverShell_PutOutputSeparator(shell);
			CyberDiverShell_PutOutputFloat(shell, CyberDiverEncoder_GetPosition(encoder));
			CyberDiverShell_PutOutputDelimiter(shell);
			return 1;
		}
	}

	return 0;
}


