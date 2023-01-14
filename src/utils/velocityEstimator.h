#ifndef __VELOCITYESTIMATOR_H_
#define __VELOCITYESTIMATOR_H_

#include <inttypes.h>

class VelocityEstimator
{
  public:
	VelocityEstimator(float interruptPeriod);
	void runIteration(int32_t steps);
	void setPulseFilterParameters(float KP, float KI, float interruptPeriod);
	float getVelocityEstimation();
	void reset();

  private:
	float interruptPeriod;
	float posError = 0.0;	  /**< Position estimation error*/
	float posEst = 0.0;		   /**< Position Estimation (Filtered Position)*/
	float velIntegrator = 0.0; /**< Velocity integrator output (Filtered velocity)*/
	float velEst = 0.0;		   /**< Estimated Velocity*/
	float pulseFilterKP = 120.0;
	float pulseFilterKI = 1900.0;
};

#endif