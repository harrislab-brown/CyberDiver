/*
 * cyber_diver_encoder_handler.h
 *
 *  Created on: Oct 28, 2024
 *      Author: johnt
 */

#ifndef INC_SHELL_HANDLERS_CYBER_DIVER_ENCODER_HANDLER_H_
#define INC_SHELL_HANDLERS_CYBER_DIVER_ENCODER_HANDLER_H_

#include "cyber_diver_shell.h"
#include "cyber_diver_encoder.h"

uint32_t CyberDiverEncoderCMD_Set(CyberDiverEncoder* encoder, CyberDiverShell* shell);
uint32_t CyberDiverEncoderCMD_Get(CyberDiverEncoder* encoder, CyberDiverShell* shell);

#endif /* INC_SHELL_HANDLERS_CYBER_DIVER_ENCODER_HANDLER_H_ */
