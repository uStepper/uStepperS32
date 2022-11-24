#ifndef __CALLBACKS_H
#define __CALLBACKS_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
	void (*_mainTimerCallback)(void);
	void (*_dropInStepInputEXTI)(void);
	void (*_dropInDirInputEXTI)(void);
	void (*_dropInEnableInputEXTI)(void);
} Callbacks_t;

extern Callbacks_t callbacks;

void mainTimerCallback();
void dropInStepInputEXTI();
void dropInDirInputEXTI();
void dropInEnableInputEXTI();
#ifdef __cplusplus
}
#endif
#endif