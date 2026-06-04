/*
 * iir.h
 *
 *  Created on: Nov 24, 2025
 *      Author: user
 */

#ifndef INC_IIR_H_
#define INC_IIR_H_

typedef struct {
    float b0, b1;      // Numerator coefficients
    float a1;          // Denominator coefficient
    float x_prev;      // Previous input
    float y_prev;      // Previous output
} IIR_filter;

/**
 * @brief  Initialize IIR filter with cutoff and sampling frequencies
 * @param  iir: Pointer to IIR_filter structure
 * @param  fc: Cutoff frequency in Hz
 * @param  fs: Sampling frequency in Hz
 * @retval None
 */
void IIR_Init(IIR_filter *iir, float fc, float fs);

/**
 * @brief  Update IIR filter with new input sample
 * @param  iir: Pointer to IIR_filter structure
 * @param  input: New input sample
 * @retval Filtered output value
 */
float IIR_Update(IIR_filter *iir, float input);

#endif /* INC_IIR_H_ */
