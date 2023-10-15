#ifndef __DROPIN_H
#define __DROPIN_H

#include <inttypes.h>
#include "../UstepperS32.h"

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
	bool invert;		 /**< Inversion of the "direction" input in dropin mode. 0 = NOT invert, 1 = invert	*/
	uint8_t holdCurrent; /**< Current to use when the motor is NOT rotating. 0-100 %	*/
	uint8_t runCurrent;  /**< Current to use when the motor is rotating. 0-100 %	*/
	uint8_t checksum;	/**< Checksum	*/
	uint16_t externalStepSize; /**< Size of each external pulse on the step pin */
} dropinCliSettings_t;

class Dropin
{
  public:
	Dropin();
	void init(uint16_t stepSize);
	bool loadDropinSettings(void);
	void saveDropinSettings(void);
	/**
	 * @brief      	This method is used to print the dropinCli menu explainer:
	 *				
	 * 				Usage:
	 *				Show this command list: 'help;'
	 *				Get PID Parameters: 'parameters;'
	 *				Set Proportional constant: 'P=10.002;'
	 *				Set Integral constant: 'I=10.002;'
	 *				Set Differential constant: 'D=10.002;'
	 *				Invert Direction: 'invert;'
	 *				Get Current PID Error: 'error;'
	 *				Get Run/Hold Current Settings: 'current;'
	 *				Set Run Current (percent): 'runCurrent=50.0;'
	 *				Set Hold Current (percent): 'holdCurrent=50.0;'	
	 *
	 */
	void printHelp();
	/**
	 * @brief      	This method is used to tune Drop-in parameters.
	 *				After tuning uStepper S, the parameters are saved in EEPROM
	 *				
	 * 				Usage:
	 *				Set Proportional constant: 'P=10.002;'
	 *				Set Integral constant: 'I=10.002;'
	 *				Set Differential constant: 'D=10.002;'
	 *				Invert Direction: 'invert;'
	 *				Get Current PID Error: 'error;'
	 *				Get Run/Hold Current Settings: 'current;'
	 *				Set Run Current (percent): 'runCurrent=50.0;'
	 *				Set Hold Current (percent): 'holdCurrent=50.0;'	
	 *
	 */
	void cli();
	/**
	 * @brief      	This method is used to change the PID proportional parameter P.
	 *
	 * @param[in]  	P - PID proportional part P
	 *
	 */
	void setProportional(float P); 

	/**
	 * @brief      	This method is used to change the PID integral parameter I.
	 *
	 * @param[in]  	I - PID integral part I
	 *
	 */
	void setIntegral(float I); 

	/**
	 * @brief      	This method is used to change the PID differential parameter D.
	 *
	 * @param[in]  	D - PID differential part D
	 *
	 */
	void setDifferential(float D); 

	/**
	 * @brief      	This method is used to invert the drop-in direction pin interpretation.
	 *
	 * @param[in]  	invert - 0 = not inverted, 1 = inverted
	 *
	 */
	void invertDir(bool invert);
	int32_t getError();

  private:
	dropinCliSettings_t settings;
	GPIO stepPin;
	GPIO dirPin;
	GPIO enaPin;
	volatile int32_t stepCnt;
	bool correctionEngaged;
	float accumError;
	VelocityEstimator externalStepFrequencyEstimator;
	
	/**
	 * @brief      	This method is used for the dropinCli to take in user commands.
	 *
	 * @param[in]  	cmd - input from terminal for dropinCli
	 *			
	 */
	void parseCommand(String *cmd);
	uint8_t settingsCalcChecksum(dropinCliSettings_t *settings);
	friend void dropInStepInputEXTI();
	friend void dropInDirInputEXTI();
	friend void dropInEnableInputEXTI();
	friend void dropInHandler();
	friend class TMC5130;
};

#endif // !__DROPIN_H
