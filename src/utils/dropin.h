#ifndef __DROPIN_H
#define __DROPIN_H

#include <inttypes.h>

/**
 * @brief      	Struct to store dropin settings
 *
 *				This struct contains the current dropin settings, aswell as a checksum,
 *				which is used upon loading of settings from EEPROM, to determine if the 
 *				settings in the EEPROM are valid.								
 * 
 */
typedef struct
{
	float P;			 /**< Proportional gain of the dropin PID controller	*/
	float I;			 /**< Integral gain of the dropin PID controller	*/
	float D;			 /**< Differential gain of the dropin PID controller	*/
	uint8_t invert;		 /**< Inversion of the "direction" input in dropin mode. 0 = NOT invert, 1 = invert	*/
	uint8_t holdCurrent; /**< Current to use when the motor is NOT rotating. 0-100 %	*/
	uint8_t runCurrent;  /**< Current to use when the motor is rotating. 0-100 %	*/
	uint8_t checksum;	/**< Checksum	*/
} dropinCliSettings_t;

#endif // !__DROPIN_H
