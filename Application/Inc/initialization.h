//
// Created by Xuleo on 8/16/2026.
//

#ifndef LEO_FOC_INITIALIZATION_H
#define LEO_FOC_INITIALIZATION_H

#include <application.h>

extern volatile uint8_t CalibrateADCFlag;
extern volatile uint8_t CalibrationElectricAngleSignal;
extern volatile uint8_t FindElectricAngleFlag;
extern volatile uint8_t FindEncoderDirectionFlag;
extern volatile float ElectricalMechanicalOffset;
extern volatile float AngleSinSum;
extern volatile float AngleCosSum;
extern volatile uint32_t DirectionPositiveCounter;
extern volatile uint32_t DirectionNegativeCounter;
extern volatile float EncoderDirection;
extern volatile uint8_t DoRotateElectricAngle; // do some rotation to overcome friction

void Init();

#endif //LEO_FOC_INITIALIZATION_H
