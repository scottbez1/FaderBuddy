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
