#include "semaphore.h"

Semaphore::Semaphore()
{
	
}

bool Semaphore::getLock()
{
	if (lock)
	{
		return false;
	}

	lock = true;
	return lock;
}

bool Semaphore::isLocked()
{
	return lock;
}

void Semaphore::releaseLock()
{
	lock = false;
}
