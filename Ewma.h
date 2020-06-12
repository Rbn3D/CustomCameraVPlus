/*
 * EWMA Filter - Exponentially Weighted Moving Average filter used for smoothing data series readings.
 *
 *     output = alpha * reading + (1 - alpha) * lastOutput
 *
 * Where:
 *  -   alpha = factor greater than 0 and less or equal to 1
 *  -   reading = current input value
 *  -   lastOutput = last filter output value
 *  -   output = filter output value after the last reading
 *
 */

#ifndef EWMA_H_
#define EWMA_H_

#include <cmath>

class Ewma {
public:
	/*
	 * Current data output
	 */
	double output = 0;

	double _alpha = .5;

	/*
	 * Creates a filter without a defined initial output. The first output will be equal to the first input.
	 */
	Ewma(double alpha);

	/*
	 * Creates a filter with a defined initial output.
	 */
	Ewma(double alpha, double initialOutput);

	void reset();

	/*
	 * Specifies a reading value.
	 * @returns current output
	 */
	double filter(double input, double dt);

private:
	bool hasInitial = false;
	double prev_input_ = 0.;
};

#endif /* EWMA_H_ */
