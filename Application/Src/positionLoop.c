//
// Created by Xuleo on 8/16/2026.
//

#include "../Inc/positionLoop.h"

PID_t Position_pid = {
    .P = 2000.0f,
    .I = 50.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

void Position_Step(const float dt) {
    float position_meters;
    if (!MT6835_GetDistanceMeters(&position_meters)) {
        return; // Or trigger encoder-fault handling
    }

    position_meters *= EncoderDirection;
    TargetDistance = TargetDistanceExternal;
    float PositionError = TargetDistance - position_meters;
    float OutputSpeed = pid_cycle(&Position_pid, PositionError, dt); // in terms of rpm
    TargetRPM = clampf(OutputSpeed, 3600.0f, -3600.0f);
}
