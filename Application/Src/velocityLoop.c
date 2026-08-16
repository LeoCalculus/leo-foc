//
// Created by Xuleo on 8/16/2026.
//

#include "../Inc/velocityLoop.h"

PID_t Velocity_pid = {
    .P = 0.8f, // if I have 1rpm error I wish it starts from Iq = 0.18A
    .I = 30.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

void Velocity_Step(const float dt) {
    // using pid to find the corresponding Iq (Id was preferred to be 0 all the time, no need setting for that)
    static uint32_t LastVelocityCount = 0;
    static uint8_t FreshVelocity = 0;
    float RPM = 0.0f;
    MT6835_Velocity_t velocity;
    if (!MT6835_GetLatestVelocity(&velocity) || velocity.sample_count == LastVelocityCount) {
        ++FreshVelocity;
        if (FreshVelocity >= 100) {
            // disable everything
            SoftStopRequested = 1;
        }
        return;
    }
    FreshVelocity = 0; // get new update reset
    LastVelocityCount = velocity.sample_count; // speed only valid when encoder get different counts
    RPM = EncoderDirection * velocity.revolutions_per_minute; // rpm
    // after getting direction, set up pid
    float VelocityError = TargetRPM - RPM;
    VelocityErrorExternal = VelocityError;
    float IqResult = pid_cycle(&Velocity_pid, VelocityError, dt);
    // also need handle stop request:
    if (!SoftStopRequested) { // if is 1 dont ovewrite target_iq
        Target_Iq = clampf(IqResult, 0.57f, -0.57f); // must within the range
    }

}
