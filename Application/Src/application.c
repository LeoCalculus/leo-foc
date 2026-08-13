//
// Created by leoxu on 8/8/26.
//

#include "../Inc/application.h"

#include "command.h"


VofaReport vofa;
Initializing FSMStates;
uint8_t DMABuffer[16];
volatile uint16_t DMAADCBusVoltage = 0;
volatile uint8_t EnableFOCStepSignal = 0;
volatile float Electric_Frequency = 5.0f;
volatile float SPWM_Modulation = 0.05f; // initial use 0.05 modulation with 5Hz
volatile uint16_t PhaseCurrent[3] = {0};
volatile uint32_t ADCOffsetCalib[3] = {0};
volatile float ADCOffset[3] = {0.0f};
volatile uint8_t DisableFOC = 0;
uint8_t UARTDMABuffer[64] = {0};
uint8_t WS2812Color[3] = {32, 0, 0};
volatile uint8_t CurrentState = EnterState; // state definition
volatile uint8_t NextState = CalibrateADC;

static volatile uint64_t AccumulatedTime = 0;
static volatile float AccumulatedTimeFoc = 0.0f; // this counter used for FOC step
static volatile uint8_t UpCountingFlag = 1;
static volatile uint8_t CalibrateADCFlag = 0; // flag used in current loop
static volatile uint16_t CalibrationCounts = 4096; // sample this many of times
static volatile uint8_t FindElectricAngleFlag = 0; // flag in current loop
static volatile uint8_t FindEncoderDirectionFlag = 0; // flag in current loop

void Init() {
    // vofa just float
    vofa.tail[0] = 0x00;
    vofa.tail[1] = 0x00;
    vofa.tail[2] = 0x80;
    vofa.tail[3] = 0x7F;
    // set ws2812 bits:
    WS2812BINARY.BIT_0 = 96; // this is 110_0000 keep low 7 bits
    WS2812BINARY.BIT_1 = 120; // this is 111_1000 keep low 7 bits
    // init for MT6835
    MT6835_Init();
    // connect dma receive:
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UARTDMABuffer, sizeof(UARTDMABuffer)-1);
    // disable UART half / full transmit
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    // enable VOFA only when all settings are done
    HAL_TIM_Base_Start_IT(&htim6);
    // start FSM:
    if (CurrentState == EnterState && NextState == CalibrateADC) {
        // update state first:
        CurrentState = CalibrateADC;
        // NextState here TODO
        CalibrateADCFlag = 1;
        // USE LED to hint:
        WS2812_SETPURE(0, 32, 0);
        WS2812_REFRESH();
        HAL_Delay(1000); // 1s is sufficient
    }

    // Blue WS2812 means the FOC is running:
    WS2812_SETPURE(0, 0, 32);
    WS2812_REFRESH();
    // Finally run FOC normally:
    DisableFOC = 0;
    // after this function will enter the main loop
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
    Electric_Frequency = 2.0f;
    SPWM_Modulation = 0.06f;

    float BusVolatge = (float)ADCBusVoltage * (3.3f/4096.0f) / 1000.0f * 16000.0f;
    vofa.data[0] = BusVolatge;
    vofa.data[1] = Electric_Frequency;
    vofa.data[2] = SPWM_Modulation;

    // the current sampling has a referece which is 1.65V
    vofa.data[3] = ((float)PhaseCurrent[0]-ADCOffset[0]) * (3.3f/4096.0f) / 50.0f / 0.001f;
    vofa.data[4] = ((float)PhaseCurrent[1]-ADCOffset[1]) * (3.3f/4096.0f) / 50.0f / 0.001f;
    vofa.data[5] = ((float)PhaseCurrent[2]-ADCOffset[2]) * (3.3f/4096.0f) / 50.0f / 0.001f;

    MT6835_Reading_t encoder;
    if (MT6835_GetLatestReading(&encoder)) {
        RotorAngle = encoder.angle_radians;
    }

    vofa.data[6] = RotorAngle;

    // update ws2812
    if (WS2812Update) {
        WS2812_SETPURE(WS2812Color[0], WS2812Color[1], WS2812Color[2]);
        WS2812_REFRESH();
        WS2812Update = 0; // only update once
    }

    // vofa send message to host PC
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)&vofa, sizeof(vofa));
}

void FOC_Step(const float dt) {
    uint16_t SnapPhaseCurrent[3];
    SnapPhaseCurrent[0] = PhaseCurrent[0];
    SnapPhaseCurrent[1] = PhaseCurrent[1];
    SnapPhaseCurrent[2] = PhaseCurrent[2];

    if (!DisableFOC) {
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

    if (CalibrateADCFlag) {
        CalibrationCounts--;
        ADCOffsetCalib[0] += SnapPhaseCurrent[0];
        ADCOffsetCalib[1] += SnapPhaseCurrent[1];
        ADCOffsetCalib[2] += SnapPhaseCurrent[2];
        if (CalibrationCounts == 0) {
            ADCOffset[0] = (float)ADCOffsetCalib[0] / 4096.0f;
            ADCOffset[1] = (float)ADCOffsetCalib[1] / 4096.0f;
            ADCOffset[2] = (float)ADCOffsetCalib[2] / 4096.0f;
            // disable flag
            CalibrateADCFlag = 0;
        }
    }
}
