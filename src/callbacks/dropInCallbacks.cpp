#include "../UstepperS32.h"
void __attribute__((optimize("O0"))) dropInStepInputEXTI()
{
	if (!ptr->dropin.correctionEngaged)
	{
		//ptr->dropin.enaPin.read() ? ptr->driver.enablePin.set() : ptr->driver.enablePin.reset();
	}
	
	bool dir = ptr->dropin.dirPin.read();
	
	if (ptr->dropin.settings.invert == dir)
	{
		ptr->dropin.stepCnt -= ptr->microSteps / ptr->dropin.settings.externalStepSize;
	}
	else
	{
		ptr->dropin.stepCnt += ptr->microSteps / ptr->dropin.settings.externalStepSize;
	}
	
}
void dropInDirInputEXTI()
{
	
}

void dropInEnableInputEXTI()
{
	if (!ptr->dropin.correctionEngaged)
	{
		//ptr->dropin.enaPin.read() ? ptr->driver.enablePin.set() : ptr->driver.enablePin.reset();
	}
}
volatile int32_t errorDebug = 0;
volatile float uDebug = 0.0;

void __attribute__((optimize("O0"))) dropInHandler()
{
	int32_t extStepCnt = ptr->dropin.stepCnt;
	
	ptr->dropin.externalStepFrequencyEstimator.runIteration(extStepCnt);
	int32_t error = extStepCnt - (int32_t)((float)ptr->encoder.getAngleMovedRaw() * ENCODERRAWTOSTEP(ptr->microSteps));
	
	float limit = abs(ptr->dropin.externalStepFrequencyEstimator.getVelocityEstimation()) + 50000.0;
	
	static float integral;
	static bool integralReset = 0;
	static float errorOld, differential = 0.0;

	float u = error*ptr->dropin.settings.P;
	
	if (u > 0.0)
	{
		if (u > limit)
		{
			u = limit;
		}
	}
	else if (u < 0.0)
	{
		if (u < -limit)
		{
			u = -limit;
		}
	}

	integral += error*ptr->dropin.settings.I;

	if (integral > 200000.0)
	{
		integral = 200000.0;
	}
	else if (integral < -200000.0)
	{
		integral = -200000.0;
	}

	if (error > -10 && error < 10)
	{
		if (!integralReset)
		{
			ptr->dropin.correctionEngaged = 0;
			//ptr->dropin.enaPin.read() ? ptr->driver.enablePin.set() : ptr->driver.enablePin.reset();
			integralReset = 1;
			integral = 0;
		}
	}
	else
	{
		ptr->dropin.correctionEngaged = 1;
		//ptr->driver.enablePin.reset();
		integralReset = 0;
	}

	u += integral;
	
	differential *= 0.9;
	differential += 0.1*((error - errorOld)*ptr->dropin.settings.D);

	errorOld = error;

	u += differential;
	
	u *= ptr->stepsPerSecondToRPM * 4.0;
	uDebug = u;
	errorDebug = error;
	ptr->setRPM(u);
	ptr->driver.setDeceleration(0xFFFE);
	ptr->driver.setAcceleration(0xFFFE);
	
}