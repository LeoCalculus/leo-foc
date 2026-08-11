//
// Created by leoxu on 8/8/26.
//

#include "../Inc/application.h"


VofaReport vofa;
uint8_t DMABuffer[16];
volatile uint16_t DMAADCBusVoltage = 0;
volatile uint8_t EnableFOCStepSignal = 0;
volatile float Electric_Frequency = 5.0f;
volatile float SPWM_Modulation = 0.05f; // initial use 0.05 modulation with 5Hz
volatile uint16_t PhaseCurrent[3] = {0};

// static data for this file only:
static volatile uint64_t AccumulatedTime = 0;
static volatile float AccumulatedTimeFoc = 0.0f; // this counter used for FOC step
static volatile uint8_t UpCountingFlag = 1;

void Init() {
    vofa.tail[0] = 0x00;
    vofa.tail[1] = 0x00;
    vofa.tail[2] = 0x80;
    vofa.tail[3] = 0x7F;
    // set ws2812 bits:
    WS2812BINARY.BIT_0 = 96; // this is 110_0000 keep low 7 bits
    WS2812BINARY.BIT_1 = 120; // this is 111_1000 keep low 7 bits
    // init for MT6835
    MT6835_Init();
}

void Application_Step(const float dt) {
    // snapshot for using variables:
    uint32_t ADCBusVoltage = DMAADCBusVoltage;
    float RotorAngle = 0.0f;

    // application of variables
    if (AccumulatedTime == 5000) {
        UpCountingFlag = 0;
    } else if (AccumulatedTime == 0){
        UpCountingFlag = 1;
    }

    if (UpCountingFlag) {
        AccumulatedTime++;
    }else {
        AccumulatedTime--;
    }

    // linear mapping for testing:
    // fe(t) = 5+9t (t has unit seconds)
    // Electric_Frequency = clampf(5.0f + 9.0f * (float)AccumulatedTime/1000.0f, 50.0f, 5.0f);
    // SPWM_Modulation = clampf(0.05f + 0.014f * (float)AccumulatedTime/1000.0f, 0.12f, 0.05f);
    Electric_Frequency = 0.0f;
    SPWM_Modulation = 0.0f;

    float BusVolatge = (float)ADCBusVoltage * (3.3f/4096.0f) / 1000.0f * 16000.0f;
    vofa.data[0] = BusVolatge;
    vofa.data[1] = Electric_Frequency;
    vofa.data[2] = SPWM_Modulation;

    // the current sampling has a referece which is 1.65V
    vofa.data[3] = ((float)PhaseCurrent[0] * (3.3f/4096.0f) - 1.6f) / 50.0f / 0.001f;
    vofa.data[4] = ((float)PhaseCurrent[1] * (3.3f/4096.0f) - 1.6f) / 50.0f / 0.001f;
    vofa.data[5] = ((float)PhaseCurrent[2] * (3.3f/4096.0f) - 1.6f) / 50.0f / 0.001f;

    MT6835_Reading_t encoder;
    if (MT6835_GetLatestReading(&encoder)) {
        RotorAngle = encoder.angle_radians;
    }

    vofa.data[6] = RotorAngle;


    if (EnableFOCStepSignal) {
        WS2812_SETPURE(0,0,32);
        WS2812_REFRESH();
    }

    // vofa send message to host PC
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)&vofa, sizeof(vofa));
}

void FOC_Step(const float dt) {
    // update dt at 20kHz still need angular velocity
    AccumulatedTimeFoc += SPWM_ANGULAR_VELOCITY_PREFIX * Electric_Frequency * dt; // 2PI * f * t
    // check if need wrap round:
    if (AccumulatedTimeFoc > 2.0f*PI) {
        AccumulatedTimeFoc -= 2.0f*PI;
    }

    float halfDuty = 4250.0f*0.5f; // sine starting position should be here

    // according to the time, find the angle offset for pwm:
    // define channel 1 has phase 0, channel 2 has phase 120 deg and channel 3 has phase -120 deg
    // initial phases
    float ThetaChannel1 = 0.0f; // 0
    float ThetaChannel2 = 2.0f * PI / 3.0f; //  120
    float ThetaChannel3 = -2.0f * PI / 3.0f; // -120

    // forminig current phase wt-theta
    float PhaseChannel1 = AccumulatedTimeFoc - ThetaChannel1;
    float PhaseChannel2 = AccumulatedTimeFoc - ThetaChannel2;
    float PhaseChannel3 = AccumulatedTimeFoc - ThetaChannel3;

    // after find phase, find corresponding duty
    // for any point on sine wave of generated pwm, point = DC bus * pwm duty, sin(phase) => duty
    // duty from 0 to 4250, 2125 is half duty, need to convert to MCU duty
    // now becomes a sine with amplitude 0 to 4250
    float Channel1Duty  = halfDuty*(1+SPWM_Modulation * sinf(PhaseChannel1));
    float Channel2Duty  = halfDuty*(1+SPWM_Modulation * sinf(PhaseChannel2));
    float Channel3Duty  = halfDuty*(1+SPWM_Modulation * sinf(PhaseChannel3));

    // N channel duty was complement setting so no need to set N channel again
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Channel1Duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Channel2Duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Channel3Duty);

    // read angle per 20KHz
    (void)MT6835_StartAngleRead_DMA();
}
