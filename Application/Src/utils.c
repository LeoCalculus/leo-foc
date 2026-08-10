//
// Created by Xuleo on 8/10/2026.
//

#include "utils.h"

float clampf(float number_to_clamp, float upper_limit, float lower_limit) {
    float output_numebr = number_to_clamp;
    if (number_to_clamp > upper_limit) {
        output_numebr = upper_limit;
    } else if (number_to_clamp < lower_limit) {
        output_numebr = lower_limit;
    }
    return output_numebr;
}