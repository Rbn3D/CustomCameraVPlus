#include "Ewma.h"

Ewma::Ewma(double alpha) {
	this->_alpha = alpha;
}

Ewma::Ewma(double alpha, double initialOutput) {
	this->_alpha = alpha;
	this->output = initialOutput;
	this->hasInitial = true;
}

void Ewma::reset() {
	this->hasInitial = false;
}

double exponentialMovingAverageIrregular(
	double alpha, double sample, double prevSample,
	double deltaTime, double emaPrev)
{
	double a = deltaTime / alpha;
	double u = exp(a * -1);
	double v = (1 - u) / a;

	double emaNext = (u * emaPrev) + ((v - u) * prevSample) +
		((1.0 - v) * sample);
	return emaNext;
}

double Ewma::filter(double input, double dt) {

	if (!hasInitial)
	{
		prev_input_ = input;
		output = input;

		hasInitial = true;
	}

	output = exponentialMovingAverageIrregular(_alpha, input, prev_input_, dt, output);

	prev_input_ = input;

	return output;
}