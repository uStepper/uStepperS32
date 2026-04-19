/**
 * Encoder Calibration Example for uStepperS32
 * 
 * This sketch demonstrates the encoder linearization/calibration feature.
 * The motor steps through 512 positions while recording encoder readings,
 * building a lookup table that corrects for non-linearities in the magnetic
 * encoder. The calibration data is stored in flash and automatically loaded
 * on subsequent boots.
 * 
 * Usage:
 *   1. Upload this sketch
 *   2. Open Serial Monitor at 115200 baud
 *   3. Send 'c' to start calibration (motor will rotate slowly)
 *   4. Send 'e' to erase stored calibration
 *   5. Send 's' to check calibration status
 *   6. Send 'r' to read current encoder angle (shows effect of linearization)
 * 
 * After calibration, all encoder readings in the library are automatically
 * linearized using the stored lookup table.
 */

#include <UstepperS32.h>

UstepperS32 stepper;

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println(F("=== uStepperS32 Encoder Calibration ==="));
    Serial.println(F("Initializing..."));

    stepper.setup(NORMAL, 200);

    if (stepper.isEncoderCalibrated())
    {
        Serial.println(F("Encoder calibration found in flash - linearization active."));
    }
    else
    {
        Serial.println(F("No calibration found. Send 'c' to calibrate."));
    }

    Serial.println(F("\nCommands:"));
    Serial.println(F("  c - Start calibration"));
    Serial.println(F("  e - Erase calibration"));
    Serial.println(F("  s - Check calibration status"));
    Serial.println(F("  r - Read encoder angle"));
}

void loop()
{
    if (Serial.available())
    {
        char cmd = Serial.read();

        switch (cmd)
        {
        case 'c':
        case 'C':
            Serial.println(F("\nStarting encoder calibration..."));
            Serial.println(F("Motor will step through 512 positions. Do not disturb!"));
            {
                bool ok = stepper.calibrateEncoder(30);
                if (ok)
                {
                    Serial.println(F("Calibration complete and saved!"));
                }
                else
                {
                    Serial.println(F("Calibration FAILED."));
                }
            }
            break;

        case 'e':
        case 'E':
            Serial.println(F("\nErasing calibration..."));
            if (stepper.eraseEncoderCalibration())
            {
                Serial.println(F("Calibration erased. Reboot to take effect."));
            }
            else
            {
                Serial.println(F("Erase failed."));
            }
            break;

        case 's':
        case 'S':
            Serial.print(F("Calibration status: "));
            Serial.println(stepper.isEncoderCalibrated() ? F("CALIBRATED") : F("NOT CALIBRATED"));
            break;

        case 'r':
        case 'R':
            Serial.print(F("Encoder angle: "));
            Serial.print(stepper.encoder.getAngleMoved(), 2);
            Serial.print(F(" deg  (raw: "));
            Serial.print(stepper.encoder.getAngleRaw());
            Serial.println(F(")"));
            break;
        }
    }

    delay(10);
}
