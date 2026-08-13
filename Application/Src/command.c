//
// Created by Xuleo on 8/12/2026.
//

#include "command.h"

uint8_t ReceivedCommandBuffer[64] = {0};
volatile uint8_t CommandReady = 0;
volatile uint8_t WS2812Update = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart4) {
        if (CommandReady) { // if here is still command ready, means last one not handled yet, drop the new command
            HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UARTDMABuffer, sizeof(UARTDMABuffer)-1);
            __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
            return;
        }
        CommandReady = 0;
        // every time receive should overwrite entire array
        memcpy(ReceivedCommandBuffer, UARTDMABuffer, Size);
        // insert "\0":
        ReceivedCommandBuffer[Size] = '\0';
        CommandReady = 1; // a signal for other function
        // restart dma
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UARTDMABuffer, sizeof(UARTDMABuffer)-1);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }
}

uint8_t NumberOfParameters(const char* argument) {
    if (argument == NULL || *argument == '\0') {
        return 0;
    }

    size_t length = strlen(argument);

    if (argument[0] == ',' || argument[length - 1] == ',' || strstr(argument, ",,") != NULL) {
        return 0;
    }

    uint8_t count = 1;

    const char *position = argument;

    while ((position = strchr(position, ',')) != NULL) {
        ++count;
        ++position;
    }

    return count;
}

void ParsingCommand(uint8_t* cmd) {
    if (CommandReady) {
        if (cmd == NULL) {
            return;
        }
        // take a snapshot:
        uint8_t Command[64];
        memcpy(Command, cmd, sizeof(Command));
        // Remove the line ending so zero-parameter commands compare exactly.
        Command[strcspn((char*)Command, "\r\n")] = '\0';

        // ready to receive the next command
        CommandReady = 0;

        // get the command first
        char *saveptr = NULL;
        char *command = strtok_r((char*)Command, ",", &saveptr);
        // no command skip
        if (command == NULL) {
            return;
        }
        // emergent stop motor and everything else
        if (strcmp(command, "ESTOP") == 0) {
            if (NumberOfParameters(saveptr) != 0) {
                return;
            }
            EmergencyStopMotor();
            return;
        }

        // stop motor smoothly
        if (strcmp(command, "STOP") == 0) {
            if (NumberOfParameters(saveptr) != 0) {
                return;
            }
            RequestMotorSoftStop();
            return;
        }

        if (strcmp(command, "WS2812") == 0) {
            if (WS2812Update) { // if last update haven't finished yet
                return;
            }
            char *probeptr = saveptr;
            if (NumberOfParameters(probeptr) != 3) {
                return; // drop failed command
            }
            char * r = strtok_r(NULL, ",", &saveptr);
            char * g = strtok_r(NULL, ",", &saveptr);
            char * b = strtok_r(NULL, ",", &saveptr);
            if (r == NULL || g == NULL || b == NULL) {
                return;
            }
            char * endptr;
            WS2812Color[0] = (uint8_t)strtol(r, &endptr, 10);
            WS2812Color[1] = (uint8_t)strtol(g, &endptr, 10);
            WS2812Color[2] = (uint8_t)strtol(b, &endptr, 10);
            WS2812Update = 1;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart4) {
        // reset buffer:
        memset(UARTDMABuffer, 0, sizeof(UARTDMABuffer));
        // restart uart
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UARTDMABuffer, sizeof(UARTDMABuffer)-1);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }
}
