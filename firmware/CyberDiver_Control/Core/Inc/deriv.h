/*
 * deriv.h
 *
 *  Created on: Aug 27, 2024
 *      Author: johnt
 */

#ifndef INC_DERIV_H_
#define INC_DERIV_H_

#include "signal.h"

/* class for derivative estimation, built on circular buffer Signal class */

typedef struct
{
	Signal signal;
	float kernel[7];
	uint32_t kernel_len;
} DerivativeEstimator;

static inline void DerivativeEstimator_Init(DerivativeEstimator* deriv)
{
	Signal_Init(&(deriv->signal));

	/*
	 * lazy but for now just hard code the kernel for estimating the derivative.
	 * these kernels were tested to work well on our encoder data in MATLAB.
	 */

	/*
	 * SG filter for first derivative using 5-sample frame and 2nd order polynomial:
	 */

//	deriv->kernel[0] = 0.2f;
//	deriv->kernel[1] = 0.1f;
//	deriv->kernel[2] = 0.0f;
//	deriv->kernel[3] = -0.1f;
//	deriv->kernel[4] = -0.2f;
//
//	deriv->kernel_len = 5;

	/*
	 * SG filter for first derivative using 7-sample frame and 2nd order polynomial:
	 */

	deriv->kernel[0] = 0.1071f;
	deriv->kernel[1] = 0.0714f;
	deriv->kernel[2] = 0.0357f;
	deriv->kernel[3] = 0.0f;
	deriv->kernel[4] = -0.0357f;
	deriv->kernel[5] = -0.0714f;
	deriv->kernel[6] = -0.1071f;

	deriv->kernel_len = 7;

}

static inline float DerivativeEstimator_Update(DerivativeEstimator* deriv, float new_value, float dt)
{
	Signal_AddDataPoint(&deriv->signal, new_value);
	return Signal_Convolve(&deriv->signal, deriv->kernel, deriv->kernel_len) / dt;
}

#endif /* INC_DERIV_H_ */
