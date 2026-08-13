//
// Created by leoxu on 8/8/26.
//

#ifndef LEO_FOC_APPLICATION_H
#define LEO_FOC_APPLICATION_H

// C lib includes
#include <stdint.h>
#include <math.h>
#include <string.h>

// STM32 includes
#include <usart.h>
#include <dma.h>
#include <adc.h>
#include "tim.h"

// USER includes
#include <ws2812.h>
#include <utils.h>
#include <mt6835.h>


// USER defines
#define PI 3.14159265358979323846f
#define TWO_PI 6.28318530717958647692f
#define SPWM_ANGULAR_VELOCITY_PREFIX (2.0f*PI) // 2*PI*f = w
#define halfDuty (4250.0f*0.5f)
#define ADCParameter ((3.3f/4096.0f) / 50.0f / 0.001f)
#define CurrentDropRate 1.0f

// structs
typedef struct VofaReport {
    float data[10];
    uint8_t tail[4];
} VofaReport;

typedef enum StartUpFSM {
    EnterState,
    CalibrateADC, // no voltage reference for INA181A2 so this is required
    FindElectricAngle, // calibrate the electric angle with mechanical angle by encoder
    FindEncoderDirection, // formula for theta_e = Direction * (pole * theta_m - theta_offset)
    ErrorState // if anything failed go to this state
} Initializing;

// create link for global usage:
extern VofaReport vofa;
extern uint8_t DMABuffer[16];
extern volatile uint16_t DMAADCBusVoltage; // float in stm32 is 32 bit
extern volatile uint8_t EnableFOCStepSignal; // this tells whether FOC_step is enabled
extern volatile float Electric_Frequency; // wt'w, w = 2PI*f, here for f
extern volatile float SPWM_Modulation;
extern volatile uint16_t PhaseCurrent[3]; // UVW
extern volatile uint32_t ADCOffsetCalib[3]; // this is in terms of ADC count
extern volatile float ADCOffset[3];
extern volatile uint8_t DisableFOC;
extern uint8_t UARTDMABuffer[64];
extern uint8_t WS2812Color[3];
extern volatile uint8_t CurrentState;
extern volatile uint8_t NextState;
extern float ClarkCurrent[2]; // Ia and Ib
extern float ClarkV[2];
extern float ParkCurrent[2]; // Id and Iq
extern float UVWCurrent[3];
extern float UVWVOut[3];
extern volatile float Theta_e;

// foc core parameters:
extern volatile float Target_Id;
extern volatile float Target_Iq;


void Init();
void Application_Step(const float dt); // 1000Hz loop
void FOC_Step(const float dt); // 20kHz loop
void RequestMotorSoftStop();
void EmergencyStopMotor();



#endif //LEO_FOC_APPLICATION_H
