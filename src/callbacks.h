#ifndef __CALLBACKS_H
#define __CALLBACKS_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
	void (*_closedLoopCallback)(void);
	void (*_mainTimerCallback)(void);
	void (*_dropInStepInputEXTI)(void);
	void (*_dropInDirInputEXTI)(void);
	void (*_dropInEnableInputEXTI)(void);
	void(*_dropInHandler)(void);
	void(*_stepMotionControllerTimerCallback)(void);
} Callbacks_t;

extern Callbacks_t callbacks;

void closedLoopCallback();
void mainTimerCallback();
void dropInStepInputEXTI();
void dropInDirInputEXTI();
void dropInEnableInputEXTI();
void dropInHandler();
void stepMotionControllerTimerCallback();
#ifdef __cplusplus
}
#endif
#endif