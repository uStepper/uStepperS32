/********************************************************************************************
* 	 	File: 		uStepperS.cpp															*
*		Version:    2.3.0                                          						    *
*      	Date: 		December 27th, 2021  	                                    			*
*      	Authors: 	Thomas Hørring Olsen                                   					*
*					Emil Jacobsen															*
*                                                   										*	
*********************************************************************************************
*	(C) 2020																				*
*																							*
*	uStepper ApS																			*
*	www.ustepper.com 																		*
*	administration@ustepper.com 															*
*																							*
*	The code contained in this file is released under the following open source license:	*
*																							*
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International			*
* 																							*
* 	The code in this file is provided without warranty of any kind - use at own risk!		*
* 	neither uStepper ApS nor the author, can be held responsible for any damage				*
* 	caused by the use of the code contained in this file ! 									*
*                                                                                           *
********************************************************************************************/
/**
* @file uStepperS.cpp
*
* @brief      Function and class implementation for the uStepper S library
*
*             This file contains class and function implementations for the library.
*
* @author     Thomas Hørring Olsen (thomas@ustepper.com)
*/
#include <UstepperSTM.h>

UstepperSTM *ptr;

void _timerCallback()
{
}

UstepperSTM::UstepperSTM() : driver(), encoder()
{
	timerCallback = _timerCallback;
}

void UstepperSTM::init()
{
	//this->encoder.init();
	this->driver.init();
}


