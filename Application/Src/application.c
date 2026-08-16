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
volatile uint16_t PhaseCurrent[3] = {0}; // order is VWU
volatile uint32_t ADCOffsetCalib[3] = {0};
volatile float ADCOffset[3] = {0.0f};
volatile uint8_t DisableFOC = 0;
uint8_t UARTDMABuffer[64] = {0};
uint8_t WS2812Color[3] = {32, 0, 0};
volatile uint8_t CurrentState = EnterState; // state definition
volatile uint8_t NextState = CalibrateADC;
float ClarkCurrent[2] = {0.0f}; // Ia and Ib
float ClarkV[2] = {0.0f};
float ParkCurrent[2] = {0.0f}; // Id and Iq
float UVWCurrent[3] = {0.0f}; // order is UVW
float UVWVOut[3] = {0.0f};
volatile float Theta_e = 0.0f;
volatile uint8_t VelocityLoopDivision = 0;
volatile uint8_t PositionLoopDivision = 0;

// safety parameters:
volatile uint8_t SoftStopRequested = 0;
static volatile uint8_t EmergencyStopLatched = 0;

static volatile float AccumulatedTimeFoc = 0.0f; // this counter used for FOC step
volatile uint8_t CalibrateADCFlag = 0; // flag used in current loop
static volatile uint16_t CalibrationCounts = 4096; // sample this many of times
volatile uint8_t CalibrationElectricAngleSignal = 0;
static volatile uint16_t CalibrationElectricAngleCounts = 4096;
volatile uint8_t FindElectricAngleFlag = 0; // flag in current loop
volatile uint8_t FindEncoderDirectionFlag = 0; // flag in current loop
volatile float ElectricalMechanicalOffset = 0.0f; // offset of encoder
volatile float AngleSinSum = 0.0f;
volatile float AngleCosSum = 0.0f;
static volatile float EncoderLastReading = 0.0f;
volatile uint32_t DirectionPositiveCounter = 0;
volatile uint32_t DirectionNegativeCounter = 0;
static volatile float AccTimeFocDirectionCalib = 0.0f;
volatile float EncoderDirection = 0;
volatile float AccumulatedMechanicalAngle = 0.0f;

// Internal signals for FOC
volatile float Target_Iq = 0.0f;
static volatile float Target_Id = 0.0f;

// tune here for current loop - in the end of Init will apply the step response
volatile float Target_Id_External = 0.0f;
volatile float Target_Iq_External = 0.5f; // 0.57 is good for application
volatile float DisplayAlphaExternal = 0.001f;
volatile float TargetRPM = 0.0f;
volatile float TargetRPMExternal = 1000.0f;
volatile float VelocityErrorExternal = 0.0f;
volatile float TargetDistance = 0.0f;
volatile float TargetDistanceExternal = 0.314f;

PID_t Id_pid = {
    .P = 0.28f,
    .I = 100.0f,
    .D = 0.0f,
    .integral_max = 0.033f
};

PID_t Iq_pid = {
    .P = 0.52f,
    .I = 100.0f,
    .D = 0.0f,
    .integral_max = 0.069f
};

// recall meaning Error = Kp * e + Ki * integral (usually max as time goes on)

void RequestMotorSoftStop() {
    if (!EmergencyStopLatched) {
        SoftStopRequested = 1;
    }
}

void EmergencyStopMotor() {
    // Disable everything and reset everything
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    DisableFOC = 1;
    EmergencyStopLatched = 1;
    SoftStopRequested = 0;
    Target_Id = 0.0f;
    Target_Iq = 0.0f;

    Id_pid.integral = 0.0f;
    Id_pid.err_m1 = 0.0f;
    Iq_pid.integral = 0.0f;
    Iq_pid.err_m1 = 0.0f;
}

void Application_Step(const float dt) {
    // need filter for display, otherwise the wave in vofa is useless
    static float FilteredIq = 0.0f;
    static float FilterRPM = 0.0f;
    const float DisplayFilterAlpha = DisplayAlphaExternal;

    // reduce the target gradually instead of sudden stop
    if (SoftStopRequested && !EmergencyStopLatched) {
        const float CurrentStep = CurrentDropRate * dt;

        if (Target_Id > CurrentStep) {
            Target_Id -= CurrentStep;
        } else if (Target_Id < -CurrentStep) {
            Target_Id += CurrentStep;
        } else { // match force 0
            Target_Id = 0.0f;
        }

        if (Target_Iq > CurrentStep) {
            Target_Iq -= CurrentStep;
        } else if (Target_Iq < -CurrentStep) {
            Target_Iq += CurrentStep;
        } else {
            Target_Iq = 0.0f;
        }

        // finally stop the motor entirely
        if (Target_Id == 0.0f && Target_Iq == 0.0f) {
            EmergencyStopMotor();
        }
    }

    vofa.data[0] = EncoderDirection;

    vofa.data[1] = TargetRPM;

    float speed_rpm;
    float speed_rpm_copy = 0.0f;
    if (MT6835_GetVelocityRPM(&speed_rpm)) {
        // speed_rpm is valid
        speed_rpm_copy = speed_rpm;
    }
    FilterRPM += DisplayFilterAlpha * (speed_rpm_copy - FilterRPM);
    vofa.data[2] = speed_rpm_copy;

    vofa.data[3] = VelocityErrorExternal;
    vofa.data[4] = FilterRPM;

    MT6835_Reading_t encoder;
    // update the struct
    MT6835_GetLatestReading(&encoder);

    vofa.data[5] = encoder.status;

    float position_meters;
    if (!MT6835_GetDistanceMeters(&position_meters)) {
        return; // Or trigger encoder-fault handling
    }

    position_meters *= EncoderDirection;

    vofa.data[6] = TargetDistance;
    vofa.data[7] = position_meters;

    // simple IIR low pass filter for display
    FilteredIq += DisplayFilterAlpha * (ParkCurrent[1] - FilteredIq);
    vofa.data[8] = Target_Iq;
    vofa.data[9] = FilteredIq;

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
    uint32_t BusVoltageCount = 0;
    BusVoltageCount = DMAADCBusVoltage;

    if (!DisableFOC) {
#ifdef VelocityLoop
        // velocity loop should in advance set up the speed information:
        if (++VelocityLoopDivision >= 5) {
#ifdef PostionLoop
            if (++PositionLoopDivision >= 4) { // so 1000Hz
                PositionLoopDivision = 0;
                Position_Step(20.0f*dt);
            }
#endif
            VelocityLoopDivision = 0;
            Velocity_Step(5.0f*dt); // 4000Hz
        }
#endif
        static float RotorAngle = 0.0f;
        // update dt at 20kHz still need angular velocity
        AccumulatedTimeFoc += SPWM_ANGULAR_VELOCITY_PREFIX * Electric_Frequency * dt; // 2PI * f * t
        // check if need wrap round:
        if (AccumulatedTimeFoc > 2.0f*PI) {
            AccumulatedTimeFoc -= 2.0f*PI;
        }

        float BusVoltage = (float)BusVoltageCount * (3.3f/4096.0f) / 1000.0f * 16000.0f;
        if (BusVoltage < 21.0f || BusVoltage > 26.0f) { // stop immediately
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            return;
        }
        // find exactly UVW current: Bro what the fuck they are negative
        // Channel 1 is U, sampling adc is ADC3
        // Channel 2 is V, sampling adc is ADC1
        // Channel 3 is W, sampling adc is ADC2
        UVWCurrent[1] = -((float)SnapPhaseCurrent[0]-ADCOffset[0]) * ADCParameter;
        UVWCurrent[2] = -((float)SnapPhaseCurrent[1]-ADCOffset[1]) * ADCParameter;
        UVWCurrent[0] = -((float)SnapPhaseCurrent[2]-ADCOffset[2]) * ADCParameter;

        // clark transform
        ClarkTransform();

        // get theta_e
        MT6835_Reading_t encoder;
        if (MT6835_GetLatestReading(&encoder)) {
            RotorAngle = encoder.angle_radians;
        }

        Theta_e = wrap2pif( EncoderDirection * (7.0f * (RotorAngle - ElectricalMechanicalOffset)));

        // do park transform:
        ParkTransform();

        // do PID to get output voltage:
        float Error_Id = Target_Id - ParkCurrent[0];
        float Error_Iq = Target_Iq - ParkCurrent[1];

        float VOutById = pid_cycle(&Id_pid, Error_Id, dt);
        float VOutByIq = pid_cycle(&Iq_pid, Error_Iq, dt);

        // do reverse Park
        ReverseParkTransform(VOutById, VOutByIq);

        // do reverse Clark to get voltage information
        ReverseClarkTransform();

        // after getting voltage, remap back to Duty:

        // Simplified, assume using exactly 24V -> power supply case
        float Channel1Duty  = (BusVoltage/2.0f + UVWVOut[0]) / BusVoltage * 4250.0f;
        float Channel2Duty  = (BusVoltage/2.0f + UVWVOut[1]) / BusVoltage * 4250.0f;
        float Channel3Duty  = (BusVoltage/2.0f + UVWVOut[2]) / BusVoltage * 4250.0f;

        // find max and min for SVPWM:
        float MaxDuty = (Channel1Duty > Channel2Duty) ? ((Channel1Duty > Channel3Duty) ? Channel1Duty : Channel3Duty) : ((Channel2Duty > Channel3Duty) ? Channel2Duty : Channel3Duty);
        float MinDuty = (Channel1Duty < Channel2Duty) ? ((Channel1Duty < Channel3Duty) ? Channel1Duty : Channel3Duty) : ((Channel2Duty < Channel3Duty) ? Channel2Duty : Channel3Duty);
        float MidDuty = (MaxDuty + MinDuty)/2.0f;
        float Voffset = halfDuty - MidDuty;

        // N channel duty was complement setting so no need to set N channel again
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, clampf(Channel1Duty+Voffset, 4250, 0));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, clampf(Channel2Duty+Voffset, 4250, 0));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, clampf(Channel3Duty+Voffset, 4250, 0));

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

    if (FindElectricAngleFlag) {
        // we assume the D axis is align with A axis, which is already aligned with U axis
        // by using current to force them align which the rotor will behave, we can record the rotate offset
        // this is the difference between mechanical and electrical offset:
        if (CalibrationElectricAngleSignal) {
            MT6835_Reading_t encoder;
            if (MT6835_GetLatestReading(&encoder)) {
                AngleSinSum += sinf(encoder.angle_radians); // accumulate like this to avoid at 0.002f and 6.282f cases
                AngleCosSum += cosf(encoder.angle_radians);
                --CalibrationElectricAngleCounts;
            }

            if (CalibrationElectricAngleCounts == 0) {
                // reset CalibEangle signal
                CalibrationElectricAngleSignal = 0;
                // also find e angle
                FindElectricAngleFlag = 0; // here finishes
            }
        }
        // we can simply set the modulation with 0 accumulated phase: (no rotatio intended so theta_e = 0)
        // assume A axis is align with U axis.
        const float CalibPhaseChannel1 = PI / 2.0f;
        const float CalibPhaseChannel2 = PI / 2.0f + 2.0f * PI / 3.0f;
        const float CalibPhaseChannel3 = PI / 2.0f - 2.0f * PI / 3.0f;

        float CalibModulation = 0.05f;

        float CalibHalfDuty = 2125.0f;
        float CalibChannel1Duty  = CalibHalfDuty*(1+CalibModulation * sinf(CalibPhaseChannel1));
        float CalibChannel2Duty  = CalibHalfDuty*(1+CalibModulation * sinf(CalibPhaseChannel2));
        float CalibChannel3Duty  = CalibHalfDuty*(1+CalibModulation * sinf(CalibPhaseChannel3));

        // N channel duty was complement setting so no need to set N channel again
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CalibChannel1Duty);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CalibChannel2Duty);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CalibChannel3Duty);

    }

    // find encoder direction need by loop
    if (FindEncoderDirectionFlag) {
        // check encoder with positive direction rotate magnetic field:
        float CurrentMechanicalReading = 0.0f;
        MT6835_Reading_t encoder;
        if (MT6835_GetLatestReading(&encoder)) {
            CurrentMechanicalReading = encoder.angle_radians;
        }
        if (CurrentMechanicalReading > EncoderLastReading) {
            DirectionPositiveCounter++;
        } else if (CurrentMechanicalReading < EncoderLastReading){
            DirectionNegativeCounter++;
        }

        // Apply positive rotate magnetic field:
        AccTimeFocDirectionCalib += TWO_PI * 5.0f * dt; // checking at 5Hz

        if (AccTimeFocDirectionCalib > 2.0f*PI) {
            AccTimeFocDirectionCalib -= 2.0f*PI;
        }

        float halfDutyCalib = 4250.0f*0.5f;

        float PhaseChannel1Calib = AccTimeFocDirectionCalib - 0.0f;
        float PhaseChannel2Calib = AccTimeFocDirectionCalib - 2.0f * PI / 3.0f;
        float PhaseChannel3Calib = AccTimeFocDirectionCalib - (-2.0f * PI / 3.0f);

        float Channel1DutyCalib  = halfDutyCalib*(1+0.05f* sinf(PhaseChannel1Calib));
        float Channel2DutyCalib  = halfDutyCalib*(1+0.05f * sinf(PhaseChannel2Calib));
        float Channel3DutyCalib  = halfDutyCalib*(1+0.05f * sinf(PhaseChannel3Calib));

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Channel1DutyCalib);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Channel2DutyCalib);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Channel3DutyCalib);

        EncoderLastReading = CurrentMechanicalReading;
    }

    (void)MT6835_StartAngleRead_DMA(); // need encoder reading at any time
}

