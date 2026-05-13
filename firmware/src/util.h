/*
 * Copyright 2026 Scott Bezek
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
