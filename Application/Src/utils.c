//
// Created by Xuleo on 8/10/2026.
//

#include "utils.h"

#define SQRT3_2 0.86602540378f

float clampf(float number_to_clamp, float upper_limit, float lower_limit) {
    float output_numebr = number_to_clamp;
    if (number_to_clamp > upper_limit) {
        output_numebr = upper_limit;
    } else if (number_to_clamp < lower_limit) {
        output_numebr = lower_limit;
    }
    return output_numebr;
}

float wrap2pif(float target) {

    target = fmodf(target, TWO_PI);

    if (target < 0.0f) {
        target += TWO_PI;
    }

    if (target >= TWO_PI) {
        target -= TWO_PI;
    }

    return target;
}

float pid_cycle(PID_t *sys, float err, const float delta_t){
    sys->integral += err*delta_t;
    if(sys->integral > sys->integral_max){
        sys->integral = sys->integral_max;
    }else if(sys->integral < -sys->integral_max){
        sys->integral = -sys->integral_max;
    }

    float derivative = (err - sys->err_m1)/delta_t;
    float output = sys->P * err + sys->I * sys->integral + sys->D * derivative;

    sys->err_m1 = err;
    return output;
}

void ClarkTransform() {
    // if not using 2/3 will have a 3/2 gain by projection
    ClarkCurrent[0] = 2.0f/3.0f * (UVWCurrent[0] - 1.0f/2.0f * UVWCurrent[1] - 1.0f/2.0f * UVWCurrent[2]); // Ia, parallel to U
    ClarkCurrent[1] = 2.0f/3.0f * (SQRT3_2 * UVWCurrent[1] - SQRT3_2 * UVWCurrent[2]); // Ib
}

void ParkTransform() {
    ParkCurrent[0] = ClarkCurrent[0] * cosf(Theta_e) + ClarkCurrent[1] * sinf(Theta_e); // Id
    ParkCurrent[1] = - ClarkCurrent[0] * sinf(Theta_e) + ClarkCurrent[1] * cosf(Theta_e); // Iq
}

void ReverseClarkTransform() {
    UVWVOut[0] = ClarkV[0];
    UVWVOut[1] = -0.5f * ClarkV[0] + SQRT3_2 * ClarkV[1];
    UVWVOut[2] = -0.5f * ClarkV[0] - SQRT3_2 * ClarkV[1];
}


void ReverseParkTransform(float VId, float VIq) {
    ClarkV[0] = VId * cosf(Theta_e) - VIq * sinf(Theta_e); // Va
    ClarkV[1] = VId * sinf(Theta_e) + VIq * cosf(Theta_e); // Vb
}