//
// Created by Xuleo on 8/12/2026.
// Purpose: Receive command from host computer
//

#ifndef LEO_FOC_COMMAND_H
#define LEO_FOC_COMMAND_H

#include <application.h> // need all signals used for control
#include <stdlib.h>

extern uint8_t ReceivedCommandBuffer[64];
extern volatile uint8_t CommandReady;
extern volatile uint8_t WS2812Update;

/*
 * Command List
 * ws2812,r,g,b  -> this set the rgb of ws2812
 *
 */

// uart dma callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void ParsingCommand(uint8_t* cmd);
uint8_t NumberOfParameters(const char* argument); // usually passsing the &safeptr

#endif //LEO_FOC_COMMAND_H
