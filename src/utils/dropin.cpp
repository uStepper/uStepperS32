#include "../UstepperS32.h"
extern UstepperS32 *ptr;

Dropin::Dropin() :	stepPin(LL_GPIO_PIN_6, 6, GPIOA),
					dirPin(LL_GPIO_PIN_2, 2, GPIOA),
					enaPin(LL_GPIO_PIN_7, 7, GPIOA),
					externalStepFrequencyEstimator(MAINTIMERINTERRUPTPERIOD)
{
	
}

void Dropin::init(uint16_t stepSize) 
{
	this->settings.externalStepSize = stepSize;
	
	this->dirPin.configureInput();
	this->stepPin.configureInterrupt(LL_GPIO_PULL_NO, LL_EXTI_TRIGGER_RISING, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
	this->enaPin.configureInterrupt(LL_GPIO_PULL_NO, LL_EXTI_TRIGGER_RISING_FALLING, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
}
void Dropin::printHelp()
{
	Serial.println(F("uStepper S Dropin !"));
	Serial.println(F(""));
	Serial.println(F("Usage:"));
	Serial.println(F("Show this command list: 'help;'"));
	Serial.println(F("Get PID Parameters: 'parameters;'"));
	Serial.println(F("Set Proportional constant: 'P=10.002;'"));
	Serial.println(F("Set Integral constant: 'I=10.002;'"));
	Serial.println(F("Set Differential constant: 'D=10.002;'"));
	Serial.println(F("Invert Direction: 'invert;'"));
	Serial.println(F("Get Current PID Error: 'error;'"));
	Serial.println(F("Get Run/Hold Current Settings: 'current;'"));
	Serial.println(F("Set Run Current (percent): 'runCurrent=50.0;'"));
	Serial.println(F("Set Hold Current (percent): 'holdCurrent=50.0;'"));
	Serial.println(F(""));
	Serial.println(F(""));
}

void Dropin::cli()
{
	static String stringInput;
	static uint32_t t = millis();

	while (1)
	{
		while (!Serial.available())
		{
			delay(1);
			if ((millis() - t) >= 500)
			{
				stringInput.remove(0);
				t = millis();
			}
		}
		t = millis();
		stringInput += (char)Serial.read();
		if (stringInput.lastIndexOf(';') > -1)
		{
			this->parseCommand(&stringInput);
			stringInput.remove(0);
		}
	}
}
void Dropin::setProportional(float P)
{
	this->settings.P = P;
}

void Dropin::setIntegral(float I)
{
	this->settings.I = I * MAINTIMERINTERRUPTPERIOD;
}

void Dropin::setDifferential(float D)
{
	this->settings.D = D * MAINTIMERINTERRUPTFREQUENCY;
}

void Dropin::invertDir(bool invert)
{
	this->settings.invert = invert;
}

bool Dropin::loadDropinSettings(void)
{
	dropinCliSettings_t tempSettings;
	//TODO: FIX THIS !
	//EEPROM.get(0, tempSettings);

	if (this->settingsCalcChecksum(&tempSettings) != tempSettings.checksum)
	{
		return 0;
	}

	this->settings = tempSettings;
	this->setProportional(this->settings.P);
	this->setIntegral(this->settings.I);
	this->setDifferential(this->settings.D);
	this->invertDir((bool)this->settings.invert);
	ptr->setCurrent(this->settings.runCurrent);
	ptr->setHoldCurrent(this->settings.holdCurrent);
	return 1;
}

void Dropin::saveDropinSettings(void)
{
	this->settings.checksum = this->settingsCalcChecksum(&this->settings);
	//TODO: FIX THIS !
	//EEPROM.put(0, this->settings);
}

uint8_t Dropin::settingsCalcChecksum(dropinCliSettings_t *settings)
{
	uint8_t i;
	uint8_t checksum = 0xAA;
	uint8_t *p = (uint8_t *)settings;

	for (i = 0; i < 15; i++)
	{
		checksum ^= *p++;
	}

	return checksum;
}

int32_t Dropin::getError()
{
	return 0;
}
void Dropin::parseCommand(String *cmd)
{
	uint8_t i = 0;
	String value;

	if (cmd->charAt(2) == ';')
	{
		Serial.println("COMMAND NOT ACCEPTED");
		return;
	}

	value.remove(0);
	/****************** SET P Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	if (cmd->substring(0, 2) == String("P="))
	{
		for (i = 2;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				Serial.print("COMMAND ACCEPTED. P = ");
				Serial.println(value.toFloat(), 4);
				this->settings.P = value.toFloat();
				this->saveDropinSettings();
				this->setProportional(value.toFloat());
				return;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}
	}

	/****************** SET I Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 2) == String("I="))
	{
		for (i = 2;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				Serial.print("COMMAND ACCEPTED. I = ");
				Serial.println(value.toFloat(), 4);
				this->settings.I = value.toFloat();
				this->saveDropinSettings();
				this->setIntegral(value.toFloat());
				return;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}
	}

	/****************** SET D Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 2) == String("D="))
	{
		for (i = 2;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				Serial.print("COMMAND ACCEPTED. D = ");
				Serial.println(value.toFloat(), 4);
				this->settings.D = value.toFloat();
				this->saveDropinSettings();
				this->setDifferential(value.toFloat());
				return;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}
	}

	/****************** invert Direction ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 6) == String("invert"))
	{
		if (cmd->charAt(6) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		if (this->settings.invert)
		{
			Serial.println(F("Direction normal!"));
			this->saveDropinSettings();
			this->invertDir(0);
			return;
		}
		else
		{
			Serial.println(F("Direction inverted!"));
			this->saveDropinSettings();
			this->invertDir(1);
			return;
		}
	}

	/****************** get Current Pid Error ********************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 5) == String("error"))
	{
		if (cmd->charAt(5) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		Serial.print(F("Current Error: "));
		Serial.print(this->getError());
		Serial.println(F(" Steps"));
	}

	/****************** Get run/hold current settings ************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 7) == String("current"))
	{
		if (cmd->charAt(7) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		Serial.print(F("Run Current: "));
		Serial.print(ceil(((float)ptr->driver.current) / 0.31));
		Serial.println(F(" %"));
		Serial.print(F("Hold Current: "));
		Serial.print(ceil(((float)ptr->driver.holdCurrent) / 0.31));
		Serial.println(F(" %"));
	}

	/****************** Get PID Parameters ***********************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 10) == String("parameters"))
	{
		if (cmd->charAt(10) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		Serial.print(F("P: "));
		Serial.print(this->settings.P, 4);
		Serial.print(F(", "));
		Serial.print(F("I: "));
		Serial.print(this->settings.I, 4);
		Serial.print(F(", "));
		Serial.print(F("D: "));
		Serial.println(this->settings.D, 4);
	}

	/****************** Help menu ********************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 4) == String("help"))
	{
		if (cmd->charAt(4) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		this->printHelp();
	}

	/****************** SET run current ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 11) == String("runCurrent="))
	{
		for (i = 11;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				if (value.toFloat() >= 0.0 && value.toFloat() <= 100.0)
				{
					i = (uint8_t)value.toFloat();
					break;
				}
				else
				{
					Serial.println("COMMAND NOT ACCEPTED");
					return;
				}
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		Serial.print("COMMAND ACCEPTED. runCurrent = ");
		Serial.print(i);
		Serial.println(F(" %"));
		this->settings.runCurrent = i;
		this->saveDropinSettings();
		ptr->setCurrent(i);
	}

	/****************** SET run current ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 12) == String("holdCurrent="))
	{
		for (i = 12;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				if (value.toFloat() >= 0.0 && value.toFloat() <= 100.0)
				{
					i = (uint8_t)value.toFloat();
					break;
				}
				else
				{
					Serial.println("COMMAND NOT ACCEPTED");
					return;
				}
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		Serial.print("COMMAND ACCEPTED. holdCurrent = ");
		Serial.print(i);
		Serial.println(F(" %"));
		this->settings.holdCurrent = i;
		this->saveDropinSettings();
		ptr->setHoldCurrent(i);
	}

	/****************** DEFAULT (Reject!) ************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else
	{
		Serial.println("COMMAND NOT ACCEPTED");
		return;
	}
}