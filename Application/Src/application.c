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
float UVWCurrentNow[3] = {0.0f};
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
volatile float Target_Id = 0.0f;

// tune here for current loop - in the end of Init will apply the step response
volatile float Target_Id_External = 0.0f;
volatile float Target_Iq_External = 0.6f; // 0.6 is good for application
volatile float DisplayAlphaExternal = 0.1f;
volatile float TargetRPM = 0.0f;
volatile float TargetRPMExternal = 0.0f;
volatile float VelocityErrorExternal = 0.0f;
volatile float TargetDistance = 0.0f;
volatile float TargetDistanceExternal = 0.314f;

// monitor static value:
static volatile float SumCurrentNow = 0.0f;
static volatile float SumCurrentHold = 0.0f;
static volatile float IqNow, IqHold = 0.0f;
static volatile float IdNow, IdHold = 0.0f;
static volatile float DutyNow, DutyHold = 0.0f;
static volatile uint8_t SumCount = 0;
static volatile uint8_t CurrentSumCount = 0;
static volatile uint8_t DutyCount = 0;
static volatile float MechanicRadsFiltered = 0.0f;
volatile float VoutInspect[2] = {0.0f};


PID_t Id_pid = {
    .P = 0.3f,
    .I = 550.0f,
    .D = 0.0f,
    .integral_max = 0.01f
};

PID_t Id_pid_Calib = {
    .P = 1.0f,
    .I = 80.0f,
    .D = 0.0f,
    .integral_max = 0.01f
};

PID_t Iq_pid = {
    .P = 0.58f,
    .I = 580.0f,
    .D = 0.0f,
    .integral_max = 0.01f
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
    static float FilteredIq = 0.0f;
    static float FilteredId = 0.0f;
    static float RPMFilterResult = 0.0f;
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

    uint32_t BusVoltageCount = 0;
    BusVoltageCount = DMAADCBusVoltage;
    float BusVoltage = (float)BusVoltageCount * (3.3f/4096.0f) / 1000.0f * 16000.0f;

    if (BusVoltage <= 20.0f || BusVoltage >= 28.0f) {
        EmergencyStopMotor();
        WS2812_SETPURE(32, 0, 0);
        WS2812_REFRESH();
    }

    vofa.data[1] = BusVoltage;
    vofa.data[2] = Target_Iq;
    FilteredIq += 0.01f * (ParkCurrent[1] - FilteredIq);
    vofa.data[3] = FilteredIq;
    FilteredId += 0.01f * (ParkCurrent[0] - FilteredId);
    vofa.data[4] = FilteredId;
    RPMFilterResult += 0.001f * (RPMHook - RPMFilterResult);
    vofa.data[5] = RPMFilterResult;
    vofa.data[6] = TargetRPM;
    vofa.data[7] = ElectricalMechanicalOffset;

    vofa.data[8] = TargetDistance;
    vofa.data[9] = PosHook;

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
        UVWCurrentNow[1] = -((float)SnapPhaseCurrent[0]-ADCOffset[0]) * ADCParameter;
        UVWCurrentNow[2] = -((float)SnapPhaseCurrent[1]-ADCOffset[1]) * ADCParameter;
        UVWCurrentNow[0] = -((float)SnapPhaseCurrent[2]-ADCOffset[2]) * ADCParameter;


        // filter each respectively, they each have same setting so do similar stuff
        UVWCurrent[1] = CurrentIIRFilterAlpha * UVWCurrentNow[1] + (1.0f - CurrentIIRFilterAlpha) * UVWCurrent[1];
        UVWCurrent[2] = CurrentIIRFilterAlpha * UVWCurrentNow[2] + (1.0f - CurrentIIRFilterAlpha) * UVWCurrent[2];
        UVWCurrent[0] = CurrentIIRFilterAlpha * UVWCurrentNow[0] + (1.0f - CurrentIIRFilterAlpha) * UVWCurrent[0];

        SumCurrentNow = UVWCurrentNow[0] + UVWCurrentNow[1] + UVWCurrentNow[2];
        if (++SumCount >= 20) {
            SumCurrentHold = SumCurrentNow / 20.0f;
            SumCurrentNow = 0.0f; // reset
            SumCount = 0;
        }

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

        IdNow += ParkCurrent[0];
        IqNow += ParkCurrent[1];
        // hook to find averaged filter:
        if (++CurrentSumCount >= 20) {
            IdHold = IdNow / 20.0f;
            IqHold = IqNow / 20.0f;
            IqNow = 0.0f;
            IdNow = 0.0f;
            CurrentSumCount = 0;
        }

        // do PID to get output voltage:
        float Error_Id = Target_Id - ParkCurrent[0];
        float Error_Iq = Target_Iq - ParkCurrent[1];

        float VOutById = pid_cycle(&Id_pid, Error_Id, dt);
        float VOutByIq = pid_cycle(&Iq_pid, Error_Iq, dt);

        VoutInspect[0] = VOutByIq;

        // MT6835_Velocity_t velocityEncoder;
        // MT6835_GetLatestVelocity(&velocityEncoder);
        // float MechanicalRads = velocityEncoder.radians_per_second; // current
        //
        // // speed also needs low pass filer since wrap around will create a lot of
        // MechanicRadsFiltered = VelocityILoopFilterAlpha * MechanicalRads + (1.0f - VelocityILoopFilterAlpha) * MechanicRadsFiltered;
        //
        // float VdDecouple = -IdIqDecoupleGain * EncoderDirection * 7.0f * MechanicRadsFiltered * LqEstimate * 0.000001f * ParkCurrent[1];
        // VOutById += VdDecouple;

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

        DutyNow += MaxDuty;
        // hook to get max duty:
        if (++DutyCount >= 20) {

            DutyHold = DutyNow / (20.0f * 4250.0f);
            DutyNow = 0.0f;
            DutyCount = 0;
        }

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
        // const float CalibPhaseChannel1 = PI / 2.0f;
        // const float CalibPhaseChannel2 = PI / 2.0f + 2.0f * PI / 3.0f;
        // const float CalibPhaseChannel3 = PI / 2.0f - 2.0f * PI / 3.0f;
        //
        // float CalibModulation = 0.1f;
        //
        // float CalibHalfDuty = 2125.0f;
        // float CalibChannel1Duty  = CalibHalfDuty*(1+CalibModulation * sinf(CalibPhaseChannel1));
        // float CalibChannel2Duty  = CalibHalfDuty*(1+CalibModulation * sinf(CalibPhaseChannel2));
        // float CalibChannel3Duty  = CalibHalfDuty*(1+CalibModulation * sinf(CalibPhaseChannel3));


        AccumulatedTimeFoc += SPWM_ANGULAR_VELOCITY_PREFIX * Electric_Frequency * dt; // 2PI * f * t
        // check if need wrap round:
        if (AccumulatedTimeFoc > 2.0f*PI) {
            AccumulatedTimeFoc -= 2.0f*PI;
        }

        float BusVoltage = (float)BusVoltageCount * (3.3f/4096.0f) / 1000.0f * 16000.0f;
        // use current method, since id is align with a axis and U axis, so set target Id to force alignment:
        // same get current first
        UVWCurrentNow[1] = -((float)SnapPhaseCurrent[0]-ADCOffset[0]) * ADCParameter;
        UVWCurrentNow[2] = -((float)SnapPhaseCurrent[1]-ADCOffset[1]) * ADCParameter;
        UVWCurrentNow[0] = -((float)SnapPhaseCurrent[2]-ADCOffset[2]) * ADCParameter;


        // filter each respectively, they each have same setting so do similar stuff
        UVWCurrent[1] = CurrentIIRFilterAlpha * UVWCurrentNow[1] + (1.0f - CurrentIIRFilterAlpha) * UVWCurrent[1];
        UVWCurrent[2] = CurrentIIRFilterAlpha * UVWCurrentNow[2] + (1.0f - CurrentIIRFilterAlpha) * UVWCurrent[2];
        UVWCurrent[0] = CurrentIIRFilterAlpha * UVWCurrentNow[0] + (1.0f - CurrentIIRFilterAlpha) * UVWCurrent[0];

        // clark transform
        ClarkTransform();

        Theta_e = 0.0f; // force this

        // do park transform:
        ParkTransform();

        // also assume theta_e is 0
        // current two value here are fixed
        Target_Id = 1.0f;
        Target_Iq = 0.0f;

        // do PID to get output voltage:
        float Error_Id = Target_Id - ParkCurrent[0];
        float Error_Iq = Target_Iq - ParkCurrent[1];

        float VOutById = pid_cycle(&Id_pid_Calib, Error_Id, dt);
        float VOutByIq = pid_cycle(&Iq_pid, Error_Iq, dt);

        // flag hook result:
        if (DoRotateElectricAngle) {
            AccumulatedTimeFoc += SPWM_ANGULAR_VELOCITY_PREFIX * Electric_Frequency * dt; // 2PI * f * t
            // check if need wrap round:
            if (AccumulatedTimeFoc > 2.0f*PI) {
                AccumulatedTimeFoc -= 2.0f*PI;
            }
            float halfDutyCalib = 4250.0f*0.5f;

            float PhaseChannel1Calib = AccumulatedTimeFoc - 0.0f;
            float PhaseChannel2Calib = AccumulatedTimeFoc - 2.0f * PI / 3.0f;
            float PhaseChannel3Calib = AccumulatedTimeFoc - (-2.0f * PI / 3.0f);

            float Channel1DutyCalib  = halfDutyCalib*(1+0.05f* sinf(PhaseChannel1Calib));
            float Channel2DutyCalib  = halfDutyCalib*(1+0.05f * sinf(PhaseChannel2Calib));
            float Channel3DutyCalib  = halfDutyCalib*(1+0.05f * sinf(PhaseChannel3Calib));

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Channel1DutyCalib);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Channel2DutyCalib);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Channel3DutyCalib);
            return;
        }

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

    // find encoder direction need by loop
    if (FindEncoderDirectionFlag) {
        // check encoder with positive direction rotate magnetic field:
        AccumulatedTimeFoc = 0.0f; // reset here since its static var
        // help reset pid as well:
        Iq_pid.integral = 0.0f;
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

