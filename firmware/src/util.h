#pragma once

#define COUNT_OF(A) (sizeof(A) / sizeof(A[0]))

float bounded_lerp(float x, float input_min, float input_max, float output_min, float output_max) {
    // Clamp x to the input range
    if (x < input_min) {
        x = input_min;
    }
    if (x > input_max) {
        x = input_max;
    }

    // Calculate the percentage of x in the input range
    float percentage = (x - input_min) / (input_max - input_min);

    // Map the percentage to the output range
    return output_min + percentage * (output_max - output_min);
}

#define BOUNDED_LERP_UINT16(input, in_min, in_max, out_min, out_max) \
    (uint16_t)((                                                    \
        ((input) <= (in_min)) ? (out_min) :                         \
        ((input) >= (in_max)) ? (out_max) :                         \
        ((uint32_t)((input) - (in_min)) * ((out_max) - (out_min)) / \
         ((in_max) - (in_min)) + (out_min))                         \
    ))
