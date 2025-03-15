/*
 * cyber_diver_controller_handler.h
 *
 *  Created on: Oct 29, 2024
 *      Author: johnt
 */

#ifndef INC_SHELL_HANDLERS_CYBER_DIVER_CONTROLLER_HANDLER_H_
#define INC_SHELL_HANDLERS_CYBER_DIVER_CONTROLLER_HANDLER_H_

#include "cyber_diver_shell.h"
#include "cyber_diver_controller.h"

uint32_t CyberDiverControllerCMD_Execute(void* obj, CyberDiverShell* shell);
void CyberDiverControllerHandler_SendData(CyberDiverControllerDataPoint* data, CyberDiverShell* shell);

#endif /* INC_SHELL_HANDLERS_CYBER_DIVER_CONTROLLER_HANDLER_H_ */
