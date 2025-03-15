/*
 * cyber_diver_logger_handler.h
 *
 *  Created on: Oct 30, 2024
 *      Author: johnt
 */

#ifndef INC_SHELL_HANDLERS_CYBER_DIVER_LOGGER_HANDLER_H_
#define INC_SHELL_HANDLERS_CYBER_DIVER_LOGGER_HANDLER_H_

#include "cyber_diver_shell.h"
#include "cyber_diver_logger.h"
#include "cyber_diver_controller_handler.h"

uint32_t CyberDiverLoggerCMD_Execute(void* obj, CyberDiverShell* shell);
uint32_t CyberDiverLoggerSender_Execute(void* obj, CyberDiverShell* shell);

#endif /* INC_SHELL_HANDLERS_CYBER_DIVER_LOGGER_HANDLER_H_ */
