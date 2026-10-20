# Arduino firmware

Use the complete [hardware synchronization guide](../docs/hardware_sync.md) for wiring, measured calibration, flashing, arming, and operation.

`MeArmSync/MeArmSync.ino` controls four directly connected servos through the official Arduino Servo library. It starts detached and refuses to arm with the supplied uncommissioned `calibration.h`. Generate that header from your measured host calibration profile. The protocol uses checksummed lines at 115200 baud; `HELLO` and `STATE` never keep the motors enabled. A missing command heartbeat detaches the servos, removing holding torque.

The optional `MeArmSync/local_joystick.example.h` implements four potentiometer inputs and a held-enable button. Enable it only after reading its wiring and source-selection instructions in the guide. Its readings are input commands, not measurements of the robot's joints.

`ServoCommissioning/ServoCommissioning.ino` is a separate, one-servo calibration aid. It starts detached and accepts explicit `ARM pin pulse`, `NUDGE 5`, `NUDGE -5`, `KEEP`, `STATE`, and `DISARM` commands in the Arduino Serial Monitor. Its five-second watchdog also removes holding torque. Start with the horn/linkage disconnected, or support the mechanism in a fixture; generic pulse limits do not guarantee safe mechanical travel.

To verify builds using an installed Arduino CLI:

```powershell
arduino-cli core install arduino:avr
arduino-cli lib install Servo
arduino-cli compile --fqbn arduino:avr:uno arduino/MeArmSync
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old arduino/MeArmSync
arduino-cli compile --fqbn arduino:avr:uno arduino/ServoCommissioning
```

Build results are recorded in [validation.json](validation.json). Compilation confirms syntax, linkage, and memory fit. Physical wiring, pulse calibration, latency, and motion still require commissioning on your arm. No firmware was uploaded and no serial device was opened during these checks.
