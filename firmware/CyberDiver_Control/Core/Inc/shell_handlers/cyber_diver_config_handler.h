/*
 * cyber_diver_config_handler.h
 *
 *  Created on: Oct 30, 2024
 *      Author: johnt
 */

#ifndef INC_SHELL_HANDLERS_CYBER_DIVER_CONFIG_HANDLER_H_
#define INC_SHELL_HANDLERS_CYBER_DIVER_CONFIG_HANDLER_H_

#include "cyber_diver_shell.h"
#include "cyber_diver_config.h"

uint32_t CyberDiverConfigCMD_Set(CyberDiverConfig* config, CyberDiverShell* shell);
uint32_t CyberDiverConfigCMD_Get(CyberDiverConfig* config, CyberDiverShell* shell);

#endif /* INC_SHELL_HANDLERS_CYBER_DIVER_CONFIG_HANDLER_H_ */
