//
// Created by Xuleo on 8/16/2026.
//

#ifndef LEO_FOC_POSITIONLOOP_H
#define LEO_FOC_POSITIONLOOP_H

#include <application.h>

extern volatile float PosHook;

void Position_Step(const float dt); // inside FOC loop ( Velocity Loop)

#endif //LEO_FOC_POSITIONLOOP_H
