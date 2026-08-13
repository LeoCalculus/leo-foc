//
// Created by Xuleo on 8/10/2026.
//

#ifndef LEO_FOC_UTILS_H
#define LEO_FOC_UTILS_H

#include <application.h>

float clampf(float number_to_clamp, float upper_limit, float lower_limit);
float wrap2pif(float target);

#endif //LEO_FOC_UTILS_H
