# uStepper S32

The library contains support for driving the stepper S32, reading out encoder data. A few examples are included to show the functionality of the library.
The library is supported and tested with in Arduino IDE 1.8.15.

For more information, visit www.ustepper.com

## Installation

Installation is split into two parts - Hardware and Library. Both are required to use the uStepper S boards.

### Hardware Installation 

To add hardware support for uStepper in the Arduino IDE (1.8.8+) do the following:
 - Open Arduino
 - Go to "File->preferences"
 - Almost at the bottom there is a field stating: "Additional Boards Manager URLs" insert this url: https://raw.githubusercontent.com/uStepper/uStepperHardware/master/package_ustepper_index.json
 - Press OK
 - Go to "Tools->Board->Boards Manager..."
 - Go to the bottom (after it has loaded new files) select "uStepper by ON Development IVS" and press install

You have now added uStepper hardware support and should be able to select uStepper under tools -> boards.

### Library Installation

To add the uStepper S library do the following:
- Open Arduino IDE (Version 1.8.8 or above)
- Go to "Sketch->Include Library->Manage Libraries..."
- Search for "uStepper S"
- Select "uStepper S" and press install
- Close Library Manager

## Documentation
The documentation for this library can be found at the following URL:

## Known bugs

## Change Log

0.1.0:	
- Initial release

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br /><span xmlns:dct="http://purl.org/dc/terms/" property="dct:title">uStepper</span> by <a xmlns:cc="http://creativecommons.org/ns#" href="www.ustepper.com" property="cc:attributionName" rel="cc:attributionURL">ON Development</a> is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.
