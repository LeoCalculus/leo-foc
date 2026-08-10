//
// Created by leoxu on 8/8/26.
//

#include "../Inc/application.h"

VofaReport vofa;
uint8_t DMABuffer[16];
volatile uint16_t DMAADCBusVoltage = 0;

// static data for this file only:
static volatile float AccumulatedTime = 0;

void Init() {
    vofa.tail[0] = 0x00;
    vofa.tail[1] = 0x00;
    vofa.tail[2] = 0x80;
    vofa.tail[3] = 0x7F;
    // set ws2812 bits:
    WS2812BINARY.BIT_0 = 96; // this is 110_0000 keep low 7 bits
    WS2812BINARY.BIT_1 = 120; // this is 111_1000 keep low 7 bits
}

void Application_Step(const float dt) {
    // snapshot for using variables:
    uint32_t ADCBusVoltage = DMAADCBusVoltage;

    // application of variables
    AccumulatedTime += dt;
    vofa.data[0] = 2.0f*sinf(5*AccumulatedTime);

    float BusVolatge = (float)ADCBusVoltage * (3.3f/4096.0f) / 1000.0f * 16000.0f;
    vofa.data[1] = BusVolatge;

    if (BusVolatge < 18.0f) {
        WS2812_SETPURE(32, 0, 32);
        WS2812_REFRESH();
    } else {
        WS2812_SETPURE(0, 32, 0);
        WS2812_REFRESH();
    }

    // vofa send message to host PC
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)&vofa, sizeof(vofa));
}

void FOC_Step(const float dt) {

}
