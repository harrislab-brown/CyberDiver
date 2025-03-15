/*
 * calib_poly.h
 *
 *  Created on: Aug 4, 2024
 *      Author: johnt
 */

#ifndef INC_CALIB_POLY_H_
#define INC_CALIB_POLY_H_

#include <stdint.h>

#define CALIB_POLY_MAX_ORDER 100
#define CALIB_POLY_LUT_SIZE 10000

typedef struct
{
	uint32_t order;  /* polynomial order */
	float coeffs[CALIB_POLY_MAX_ORDER + 1];  /* array of coefficients */

	/* input bounds (if the input exceeds the bounds, return the polynomial evaluated at the bound) */
	float input_min;
	float input_max;

	uint32_t fast_compute;  /* whether to directly calculate or use the LUT */
} CalibrationPolynomial_Config;

typedef struct
{
	CalibrationPolynomial_Config* config;

	/* lookup table array for fast evaluation */
	float lut[CALIB_POLY_LUT_SIZE];
	float lut_spacing;

} CalibrationPolynomial;

void CalibrationPolynomial_Init(CalibrationPolynomial* calib_poly, CalibrationPolynomial_Config* config);
float CalibrationPolynomial_Eval(CalibrationPolynomial* calib_poly, float input);

void CalibrationPolynomial_ComputeLUT(CalibrationPolynomial* calib_poly);

void CalibrationPolynomial_Set(CalibrationPolynomial* calib_poly, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute);
void CalibrationPolynomial_Get(CalibrationPolynomial* calib_poly, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute);


#endif /* INC_CALIB_POLY_H_ */
