# Modbus Register Description for uStepper S32

## Connection
On uStepper S32 the Modbus interface is pre-defined to be situated at Serial 2 pins D4 and D5. The interface directly on the uStepper S32 is TTL. An RS485 transciever is required to handle the physical layer in an Modbus RTU setup.
One option is to use the uStepper S32 WiFi shield, which embeds an RS485 interface and can be programmed to use this interface with the uStepper S32.

Details on this can be found here: [uStepper S32 WiFi Shield](https://github.com/uStepper/uStepperS32WiFiShield)

## Communication Parameters
- **Protocol**: Modbus RTU
- **Baud Rate**: Configurable (default: 9600 bps)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Slave ID**: Configurable (default: 1)

---

## Sensor/Info Registers

| Name            | Address   | Scale | Unit      | Description                                      |
|------------------|-----------|-------|-----------|--------------------------------------------------|
| Encoder Angle    | 0 and 1   | 1.0   | Degrees   | 32-bit float. Range: 0.0° to 360.0°.            |
| Driver RPM       | 2 and 3   | 1.0   | RPM       | 32-bit float.                                   |
| Encoder RPM      | 4 and 5   | 1.0   | RPM       | 32-bit float.                                   |
| Motor State      | 6         | 1     | None      | 0 = Stopped, 1 = Running.                       |
| Stall Status     | 7         | 1     | None      | 0 = Not stalled, 1 = Stalled.                   |

---

## Command Registers

| Name                | Address   | Scale | Unit      | Description                                      |
|---------------------|-----------|-------|-----------|--------------------------------------------------|
| Target Angle        | 8 and 9   | 1.0   | Degrees   | 32-bit float. Used in `moveToAngle` mode.       |
| Target RPM          | 10 and 11 | 1.0   | RPM       | 32-bit float. Used in `setRPM` mode.            |
| Steps to Move       | 12 and 13 | 1.0   | Steps     | 32-bit float. Used in `moveSteps` mode.         |
| Relative Angle      | 14 and 15 | 1.0   | Degrees   | 32-bit float. Used in `moveAngle` mode.         |
| Brake Mode          | 16        | 1     | None      | 0 = HARD (immediate stop), 1 = SOFT (gradual stop), 2 = FREEWHEEL (motor spins freely), 3 = BRAKE (active braking). |
| Operation Mode      | 17        | 1     | None      | 0 = moveToAngle, 1 = setRPM, 2 = moveSteps, 3 = moveAngle. |
| Max Acceleration    | 18 and 19 | 1.0   | RPM/s     | 32-bit float. Rrecommended range: 0 RPM/s to 330 RPM/s.    |
| Max Velocity        | 20 and 21 | 1.0   | RPM       | 32-bit float. Recommended range: 0 RPM to 1000 RPM.      |
| Loop Mode           | 22        | 1     | None      | 0 = Disabled, 1 = Enabled.                      |
| Closed Loop Mode    | 22        | 1     | None      | 0 = Disabled (open-loop control), 1 = Enabled (closed-loop control). |
| Running Current     | 24 and 25 | 1.0   | Percent   | 32-bit float. Range: 0% to 100% of 2.0 A.       |

---

## Notes
1. **Data Encoding**: 
   - 32-bit floating-point values are stored in two consecutive 16-bit holding registers. The high part is stored in the first register, and the low part is stored in the second register.
   - Integer values are stored in a single 16-bit holding register.

2. **Running Current**:
   - The running current is stored in registers 24 and 25 as a 32-bit float.
   - **Range**: 0% to 100% of the maximum current (2.0 A).
   - **Unit**: Percent (%). For example, 50% corresponds to 1.0 A.

3. **Operation Modes**:
   - `moveToAngle` (Mode 0): Moves the motor to a specific angle. The target angle is stored in registers 8 and 9.
   - `setRPM` (Mode 1): Sets the motor's RPM. The target RPM is stored in registers 10 and 11.
   - `moveSteps` (Mode 2): Moves the motor a specific number of steps. The step count is stored in registers 12 and 13.
   - `moveAngle` (Mode 3): Moves the motor by a relative angle. The relative angle is stored in registers 14 and 15.

4. **Closed Loop Mode**:
   - Closed loop mode is stored in register 22.
   - **Modes**:
     - 0 = Disabled: The motor operates in open-loop control.
     - 1 = Enabled: The motor operates in closed-loop control, using feedback from the encoder to adjust its position or speed.

5. **Max Velocity and Acceleration**:
   - Maximum velocity is stored in registers 20 and 21 as a 32-bit float.
   - Maximum acceleration is stored in registers 18 and 19 as a 32-bit float.
   - **Units**: Velocity is in RPM, and acceleration is in RPM/s.

6. **Encoder Angle**:
   - The encoder angle is stored in registers 0 and 1 as a 32-bit float.
   - **Range**: 0.0° to 360.0° (wraps around after 360°).
   - **Unit**: Degrees (°).

7. **Brake Modes**:
   - Brake mode is stored in register 16.
   - **Modes**:
     - 0 = HARD (immediate stop).
     - 1 = SOFT (gradual stop).
     - 2 = FREEWHEEL (motor spins freely).
     - 3 = BRAKE (active braking).

8. **Initialization**:
   - Use the `modbusEnable` function to initialize Modbus communication with the desired baud rate and slave ID.

9. **Register Updates**:
   - Registers 0–7 are updated dynamically based on the motor's state, encoder readings, and stall status.