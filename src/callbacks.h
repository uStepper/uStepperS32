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
} Callbacks_t;

extern Callbacks_t callbacks;

void closedLoopCallback();
void mainTimerCallback();
void dropInStepInputEXTI();
void dropInDirInputEXTI();
void dropInEnableInputEXTI();
void dropInHandler();
#ifdef __cplusplus
}
#endif
#endif