/*
 * iir.c
 *
 *  Created on: Nov 24, 2025
 *      Author: user
 */


#include "iir.h"

#define PI 3.14159265359f

/**
 * @brief  Initialize IIR filter coefficients using bilinear transform
 * @param  iir: Pointer to IIR_filter structure
 * @param  fc: Cutoff frequency in Hz
 * @param  fs: Sampling frequency in Hz
 * @retval None
 *
 * Design method: Bilinear transformation of analog RC filter
 * Analog: H(s) = ωc / (s + ωc)
 * Digital: Bilinear transform s = (2/T) * (1-z^-1)/(1+z^-1)
 */
void IIR_Init(IIR_filter *iir, float fc, float fs) {
    float omega = 2.0f * PI * fc;  // Angular cutoff frequency
    float T = 1.0f / fs;            // Sampling period
    float a0 = 2.0f + (omega * T);  // Normalization factor

    // Calculate filter coefficients
    iir->b0 = (omega * T) / a0;
    iir->b1 = iir->b0;
    iir->a1 = ((omega * T) - 2.0f) / a0;

    // Initialize state variables
    iir->x_prev = 0.0f;
    iir->y_prev = 0.0f;
}

/**
 * @brief  Process one sample through IIR filter
 * @param  iir: Pointer to IIR_filter structure
 * @param  input: New input sample
 * @retval Filtered output value
 *
 * Difference equation: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
 * Optimized: Uses Direct Form I structure for numerical stability
 */
float IIR_Update(IIR_filter *iir, float input) {
    // Calculate output using difference equation
    float output = (iir->b0 * input)
                 + (iir->b1 * iir->x_prev)
                 - (iir->a1 * iir->y_prev);

    // Update state variables for next iteration
    iir->x_prev = input;
    iir->y_prev = output;

    return output;
}
