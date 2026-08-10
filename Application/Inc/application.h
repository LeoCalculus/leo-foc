//
// Created by leoxu on 8/8/26.
//

#ifndef LEO_FOC_APPLICATION_H
#define LEO_FOC_APPLICATION_H

// C lib includes
#include <stdint.h>
#include <math.h>

// STM32 includes
#include <usart.h>
#include <dma.h>
#include <adc.h>

// USER includes
#include <ws2812.h>
#include <utils.h>

// USER defines
#define PI 3.14159265f  // define PI a float here
#define SPWM_ANGULAR_VELOCITY_PREFIX (2.0f*PI) // 2*PI*f = w

// structs
typedef struct VofaReport {
    float data[10];
    uint8_t tail[4];
} VofaReport;


// create link for global usage:
extern VofaReport vofa;
extern uint8_t DMABuffer[16];
extern volatile uint16_t DMAADCBusVoltage; // float in stm32 is 32 bit
extern volatile uint8_t EnableFOCStepSignal; // this tells whether FOC_step is enabled
extern volatile float Electric_Frequency; // wt'w, w = 2PI*f, here for f
extern volatile float SPWM_Modulation;

void Init();
void Application_Step(const float dt); // 1000Hz loop
void FOC_Step(const float dt); // 20kHz loop



#endif //LEO_FOC_APPLICATION_H
