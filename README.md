# uStepper S32

The library contains support for driving the stepper S32, reading out encoder data. A few examples are included to show the functionality of the library.
The library is supported and tested with in Arduino IDE 1.8.15.

For more information, visit www.ustepper.com

## How to program

**Follow this after completing the installation section further down.**

Since uStepper S32 uses an STM32 MCU the procedure for programming involves some specific steps.

When programming attach your uStepper S32 board to your PC and verify it shows up under "port" in the tools menu.
Load your program and compile it.
 
Once error free do as follows:
 - Press and hold down the "boot" switch.
 - Press the "reset" switch and release it again and then.
 - Release the "boot" switch.

Now press upload in the Arduino IDE and the program will upload.

## Installation

Installation is split into two parts - Hardware and Library. Both are required to use the uStepper S32 boards.
A video tutorial walking you through the below steps can be found here: https://youtu.be/CyVZvoLPSM4

### Hardware Installation 

First of you need to install the STM32 CUBE Programmer from here: https://www.st.com/en/development-tools/stm32cubeprog.html

To add hardware support for uStepper in the Arduino IDE (1.8.8+) do the following:
 - Open Arduino
 - Go to "File->preferences"
 - Almost at the bottom there is a field stating: "Additional Boards Manager URLs" insert this url: https://raw.githubusercontent.com/uStepper/uStepperSTM32Hardware/master/package.json
 - Press OK
 - Go to "Tools->Board->Boards Manager..."
 - Go to the bottom (after it has loaded new files) select "uStepper STM32 boards" and press install

You have now added uStepper STM32 hardware support and should be able to select uStepper STM32 boards under tools -> boards.

### Library Installation

To add the uStepper S32 library do the following:
- Open Arduino IDE (Version 1.8.8 or above)
- Go to "Sketch->Include Library->Manage Libraries..."
- Search for "uStepper S32"
- Select "uStepper S32" and press install
- Close Library Manager

## Documentation
The documentation for this library can be found at the following URL:

## Known bugs

## Change Log
0.1.1:	
- Fixed wifiGui example

0.1.0:	
- Initial release

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br /><span xmlns:dct="http://purl.org/dc/terms/" property="dct:title">uStepper</span> by <a xmlns:cc="http://creativecommons.org/ns#" href="www.ustepper.com" property="cc:attributionName" rel="cc:attributionURL">ON Development</a> is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.
