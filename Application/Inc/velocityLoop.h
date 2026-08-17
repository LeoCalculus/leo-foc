//
// Created by Xuleo on 8/16/2026.
//

#ifndef LEO_FOC_VELOCITYLOOP_H
#define LEO_FOC_VELOCITYLOOP_H

#include <application.h>

extern volatile uint8_t SoftStopRequested;
extern volatile float Target_Iq;
extern volatile float Target_Id;
extern volatile float RPMFiltered;
extern volatile float RPMHook;

void Velocity_Step(const float dt); // inside FOC step but using 4KHz

#endif //LEO_FOC_VELOCITYLOOP_H
