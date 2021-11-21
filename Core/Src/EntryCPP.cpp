/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "CommManager.hpp"
#include "ServoControl.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;

CommManager CommunicationManager;
ServoControl Servos(&htim2);

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;

void main_cpp(void)
{
//	MessageInfoTypeDef msg = {0};
//	msg.
//	CommunicationManager.AttachCommInt(&huart4, &hdma_uart4_rx, &hdma_uart4_tx);
//	CommunicationManager.PushCommRequestIntoQueue(MsgInfo)
	Servos
}


#ifdef __cplusplus
}
#endif
