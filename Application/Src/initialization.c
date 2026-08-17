//
// Created by Xuleo on 8/16/2026.
//

#include "../Inc/initialization.h"

volatile uint8_t DoRotateElectricAngle = 0;

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
    MT6835_SetVelocityFilterCutoff(20.0f); // encoder is very noisy must use filter
    // connect dma receive:
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UARTDMABuffer, sizeof(UARTDMABuffer)-1);
    // disable UART half / full transmit
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    // enable VOFA only when all settings are done
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_Delay(500);
    // start FSM, encoder must be enabled at this stage
    if (CurrentState == EnterState && NextState == CalibrateADC) {
        // update state first:
        CurrentState = CalibrateADC;
        NextState = FindElectricAngle;
        // check BUS voltage first
        uint32_t BusVoltageCount = 0;
        BusVoltageCount = DMAADCBusVoltage;
        float BusVoltage = (float)BusVoltageCount * (3.3f/4096.0f) / 1000.0f * 16000.0f;
        if (BusVoltage <= 20.0f || BusVoltage >= 28.0f) {
            NextState = ErrorState; // Bus Voltage Error
        } else {
            CalibrateADCFlag = 1; // flag turned off inside FOC loop
            // USE LED to hint:
            WS2812_SETPURE(0, 32, 0);
            WS2812_REFRESH();
            HAL_Delay(1000); // 1s is sufficient
        }
    }

    // find electric angle need rotate and then attach
    if (CurrentState == CalibrateADC && NextState == FindElectricAngle) {
        CurrentState = FindElectricAngle;
        NextState = FindEncoderDirection;
        FindElectricAngleFlag = 1; // flag turned off below
        // USE LED to hint
        WS2812_SETPURE(32, 32, 0); // yellow for finding angle offset
        WS2812_REFRESH();
        // in next 1s do some rotation:
        DoRotateElectricAngle = 1;
        HAL_Delay(1000); // actually should be less than 2.5 seconds
        DoRotateElectricAngle = 0; // shut down and start attach immediately
        HAL_Delay(1000); // another 1s wait for rotor to stop
        // deal with encoder offset
        CalibrationElectricAngleSignal = 1; // start sample 4096 times
        HAL_Delay(500);
        ElectricalMechanicalOffset = atan2f(AngleSinSum, AngleCosSum); // self division to get the average result, this number should within 0 to 2pi
        if (ElectricalMechanicalOffset < 0.0f) {
            ElectricalMechanicalOffset += TWO_PI;
        }
        FindElectricAngleFlag = 0; // disable the flag
    }

    // lastly find the encoder increment information:
    if (CurrentState == FindElectricAngle && NextState == FindEncoderDirection) {
        CurrentState = FindEncoderDirection;
        FindEncoderDirectionFlag = 1;
        // apply positive direction magnetic field:
        WS2812_SETPURE(25, 16, 0); // yellow for finding angle offset
        WS2812_REFRESH();
        HAL_Delay(2000); // delay just show led, actual run should within this time
        if (DirectionPositiveCounter > DirectionNegativeCounter) { // they will not equal unless motor not moving
            EncoderDirection = 1.0f;
        } else if (DirectionPositiveCounter < DirectionNegativeCounter) {
            EncoderDirection = -1.0f;
        } // if equal just keep 0.0f
        FindEncoderDirectionFlag = 0;
    }

    // this means the error state from Bus Voltage Reading
    if (CurrentState == CalibrateADC && NextState == ErrorState) {
        WS2812_SETPURE(32, 0, 0); // yellow for finding angle offset
        WS2812_REFRESH();
        EmergencyStopMotor(); // disable everything and wait for reset
        return; // dont execute following code
    }

    HAL_Delay(2000);
    // Handler now gives to velocity loop instead
#ifdef PostionLoop
    MT6835_ResetTotalAngleCounts();
    TargetDistance = TargetDistanceExternal;
#endif
    // reset some global vars:
    // Target_Iq = Target_Iq_External; // so can debug easier
    // Target_Id = 0.0f;
    // reset pid for Iq:
    // TargetRPM = TargetRPMExternal;
    // Target_Id = Target_Id_External;
    TargetDistance = TargetDistanceExternal;
    // Blue WS2812 means the FOC is running:
    WS2812_SETPURE(0, 0, 32);
    WS2812_REFRESH();
    // Finally run FOC normally:
    DisableFOC = 0;
    // after this function will enter the main loop
}
