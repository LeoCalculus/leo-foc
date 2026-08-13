//
// Created by Xuleo on 8/10/2026.
//

#ifndef LEO_FOC_UTILS_H
#define LEO_FOC_UTILS_H

#include <application.h>

typedef struct PID_t{
    const float P;
    const float I;
    const float D;
    const float integral_max;
    float integral;
    float err_m1;
}PID_t;

float clampf(float number_to_clamp, float upper_limit, float lower_limit);
float wrap2pif(float target);
void ClarkTransform();
void ParkTransform();
void ReverseClarkTransform();
void ReverseParkTransform(float VId, float VIq);
float pid_cycle(PID_t *sys, float err, const float delta_t);

#endif //LEO_FOC_UTILS_H
