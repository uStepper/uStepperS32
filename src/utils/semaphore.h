#ifndef __SEMAPHORE_H
#define __SEMAPHORE_H

class Semaphore
{
  public:
	Semaphore();

	bool getLock();
	bool isLocked();
	void releaseLock();

  private:
	bool lock = 0;
};

#endif