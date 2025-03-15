/*
 * calib_poly.c
 *
 *  Created on: Aug 4, 2024
 *      Author: johnt
 */

#include "calib_poly.h"


static float CalibrationPolynomial_DirectEval(CalibrationPolynomial* calib_poly, float input)
{
	/* clamp the input */
	input = input < calib_poly->config->input_min ? calib_poly->config->input_min : input;
	input = input > calib_poly->config->input_max ? calib_poly->config->input_max : input;

	/* evaluate the polynomial */
	float ret = 0.0f;

	for (uint32_t i = 0; i <= calib_poly->config->order; i++)
	{
		float pow = 1.0f;

		for (uint32_t j = 0; j < i; j++)
		{
			pow *= input;
		}

		ret += pow * calib_poly->config->coeffs[i];
	}

	return ret;
}


void CalibrationPolynomial_Init(CalibrationPolynomial* calib_poly, CalibrationPolynomial_Config* config)
{
	calib_poly->config = config;

	CalibrationPolynomial_ComputeLUT(calib_poly);
}


float CalibrationPolynomial_Eval(CalibrationPolynomial* calib_poly, float input)
{

	if (calib_poly->config->fast_compute)
	{
		/* input bounds checking */
		if (input <= calib_poly->config->input_min)
			return calib_poly->lut[0];
		if (input >= calib_poly->config->input_max)
			return calib_poly->lut[CALIB_POLY_LUT_SIZE - 1];

		/* determine the table index */
		uint32_t index = (input - calib_poly->config->input_min) / calib_poly->lut_spacing;

		/* not doing any interpolation for highest speed */
		return calib_poly->lut[index];
	}
	else
	{
		return CalibrationPolynomial_DirectEval(calib_poly, input);
	}
}

void CalibrationPolynomial_ComputeLUT(CalibrationPolynomial* calib_poly)
{
	calib_poly->lut_spacing = (calib_poly->config->input_max - calib_poly->config->input_min) / ((float)CALIB_POLY_LUT_SIZE - 1.0f);
	for (uint32_t i = 0; i < CALIB_POLY_LUT_SIZE; i++)
	{
		float input = (float)i * calib_poly->lut_spacing + calib_poly->config->input_min;
		calib_poly->lut[i] = CalibrationPolynomial_DirectEval(calib_poly, input);
	}
}

void CalibrationPolynomial_Set(CalibrationPolynomial* calib_poly, uint32_t order, float* coeffs, float input_min, float input_max, uint32_t fast_compute)
{
	/* validation */
	if (order > CALIB_POLY_MAX_ORDER) order = CALIB_POLY_MAX_ORDER;
	if (input_min >= input_max) return;

	calib_poly->config->fast_compute = 0;  /* switch to direct evaluation while we compute the new lookup table */

	/*
	 * If we try to evaluate the polynomial while we are setting the new parameters we could get weird results, but it shouldn't outright break
	 */

	calib_poly->config->input_min = input_min;
	calib_poly->config->input_max = input_max;
	calib_poly->config->order = order;
	for (uint32_t i = 0; i <= calib_poly->config->order; i++)
		calib_poly->config->coeffs[i] = coeffs[i];

	CalibrationPolynomial_ComputeLUT(calib_poly);

	calib_poly->config->fast_compute = fast_compute ? 1 : 0;

}

void CalibrationPolynomial_Get(CalibrationPolynomial* calib_poly, uint32_t* order, float** coeffs, float* input_min, float* input_max, uint32_t* fast_compute)
{
	*order = calib_poly->config->order;
	*coeffs = calib_poly->config->coeffs;
	*input_min = calib_poly->config->input_min;
	*input_max = calib_poly->config->input_max;
	*fast_compute = calib_poly->config->fast_compute;
}
