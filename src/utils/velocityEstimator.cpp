#include "velocityEstimator.h"

VelocityEstimator::VelocityEstimator(float interruptPeriod)
{
	this->interruptPeriod = interruptPeriod;
}

void VelocityEstimator::setPulseFilterParameters(float KP, float KI, float interruptPeriod)
{
	this->pulseFilterKP = KP;
	this->pulseFilterKI = KI;
	this->interruptPeriod = interruptPeriod;
}

void VelocityEstimator::runIteration(int32_t steps)
{
	this->posEst += this->velEst * this->interruptPeriod;
	this->posError = (float)steps - this->posEst;
	this->velIntegrator += this->posError * this->pulseFilterKI * this->interruptPeriod;
	this->velEst = (this->posError * this->pulseFilterKP) + this->velIntegrator;
}

void VelocityEstimator::reset()
{
	this->posEst = 0.0;
	this->posError = 0.0;
	this->velIntegrator = 0.0;
	this->velEst = 0.0;
}

float VelocityEstimator::getVelocityEstimation()
{
	return this->velIntegrator;
}