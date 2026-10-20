# Synchronizing MuJoCo with an Arduino MeArm

The bridge connects the existing Classic V1 MuJoCo model to four Arduino-driven
hobby servos over USB serial. Start with **simulation-to-hardware**: move the
MuJoCo actuator sliders or type a joint/XYZ command, and the Arduino follows the
calibrated servo targets. **Hardware-to-simulation** displays targets reported by
the Arduino when your own Arduino control logic supplies them.

This is **command synchronization**. Ordinary three-wire servos do not report
their actual shaft angle to the Arduino. A stalled servo, a loose horn, or an arm
moved while power is off will not be detected by this bridge. Arduino
`Servo.read()` and `readMicroseconds()` return the most recently written command;
they are not encoder measurements. See the
[official Servo API source](https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h).

```text
MuJoCo position targets ── calibration ── USB ── Arduino limit/slew checks ── servos
                                                │
MuJoCo position targets ◀─ inverse calibration ◀──┘ applied-command telemetry
```

The two directions are separate operating modes. One controller owns the servo
targets at a time. The simulator's joint positions can lag its targets under
physics; the real arm has its own load, friction, backlash, and servo response.

## 1. Hardware and wiring

The supplied sketch targets an Arduino Uno or classic Nano using the Arduino
Servo library. Select your actual board in Arduino IDE; other boards need their
matching core, Servo support, and valid pin assignments. A PCA9685 or another
servo shield needs its own output adapter; this sketch drives servo signal pins
directly.

| MeArm servo | Arduino signal pin | Model control |
| --- | --- | --- |
| Base | D11 | `base` / yaw |
| Shoulder | D10 | `shoulder` / upper-arm elevation |
| Elbow linkage | D9 | `elbow` / absolute forearm elevation |
| Claw | D6 | `gripper` / jaw opening |

These pins follow the
[MeArm-Arduino README example](https://github.com/MeArm/MeArm-Arduino#readme).
Check your kit's wiring rather than relying on wire colors alone.

With both supplies off, connect the four servo signal wires to these pins. Feed
the servo power wires from a regulated supply compatible with your specific
servos, commonly 5 V for MeArm micro servos. Choose current capacity from the
servos' maximum/stall current specifications for four motors and allow margin.
[Arduino's servo power guidance](https://docs.arduino.cc/learn/electronics/servo-motors/#powering-servo-motors)
explains why servo specifications and load determine the supply requirements.
Connect servo ground, supply negative, and Arduino GND together. USB powers the
Arduino and carries serial data; route the motors' power directly from their
supply. Do not connect the external supply positive to the USB-powered Arduino
5 V rail unless your board/shield explicitly supports that arrangement.
[Arduino recommends external power and a common ground for multiple servos](https://support.arduino.cc/hc/en-us/articles/360018922259-What-power-supply-can-I-use-with-my-Arduino-board).

Keep a reachable switch in the servo supply. The software stop detaches servo
signals; it does not interrupt motor power. Detaching or losing power can let the
arm fall, so support the mechanism and remove any payload during commissioning.
Mount the base firmly and clear the arm's swept volume.

## 2. Install Python and Arduino software

Use the same Windows Conda environment that already launches your simulation.
In Command Prompt or the VS Code terminal:

```bat
cd C:\Users\Luka\Documents\ChatGPT\MeArm
conda activate mearm
python -m pip install -r requirements.txt
python -m pip install -r requirements-hardware.txt
python scripts/sync_arduino.py --dry-run
```

Dry run opens the viewer without opening a serial port. It is useful for learning
the controls before connecting motors. These Python commands do not require a
PowerShell execution-policy change.

Install [Arduino IDE](https://www.arduino.cc/en/software), your board package, and
the **Servo** library through Library Manager if it is not already available.
For Uno/classic Nano, the board package is **Arduino AVR Boards**. Keep a copy of
your current working sketch so you can restore it. The synchronization sketch
replaces it when uploaded.

## 3. Measure and record calibration

Create your own calibration file:

```bat
python scripts/calibrate_hardware.py --init config/hardware_calibration.json
```

The generated example is **uncalibrated**, so it cannot enable hardware motion.
Its numbers describe a starting template, not the measured limits of your arm.
The firmware also ships with calibration disabled. Do not mark the file
calibrated just to bypass that check.

For each servo, record corresponding **model angle in degrees** and **servo
pulse width in microseconds** at a lower angle, a middle/reference angle, and a
higher angle. The mapping interpolates between adjacent measurements, allowing
different slopes on either side of the reference. At least three paired anchors
are required; more are supported. Pulse width may increase or decrease with
angle, but the mapping must be strictly monotonic. The first and last anchors
define the conservative allowed window. Also record an initial pose, the output
pin, and the maximum pulse change per second.

Use your existing known-working controller, a servo tester, or the supplied
single-servo commissioning sketch below to take the measurements. The
synchronization sketch intentionally cannot discover limits by sweeping an
uncalibrated arm.
If the horn/linkage position is unknown, remove or decouple the linkage first,
identify a known reference pulse, then refit the horn according to the kit's
assembly instructions. Take measurements with the arm supported and no cube.

1. Identify each physical motor, output pin, and its installed horn direction.
2. Find a known, mechanically clear reference pose using your existing working
   setup. Measure the upper arm and forearm against horizontal with an angle
   gauge. Record actual output pulses, not merely a nominal `Servo.write(90)`.
3. Adjust one motor at a time in small pulse increments around that reference.
   Stop well before a linkage binds, a servo buzzes against a stop, or current
   rises sharply. Never run a generic full-range sweep on the assembled arm.
4. Initially keep a narrow permitted interval around the measured pose. A few
   degrees each way is enough to verify sign and joint identity. Leave margin
   inside every mechanical limit. Set the initial pose to the supported,
   physically aligned pose you measured.
5. Measure both sides of the reference. For the claw, measure one jaw's opening
   angle relative to closed and record the gap separately for object-fit checks.
   The JSON `grip` values are degrees, not gap millimetres. Do not infer the claw
   mapping from shoulder/base geometry.
6. Replace the template values, verify all four channels, then set the calibration
   confirmation flag only after you have checked those values on your hardware.

Calibrate the **assembled mechanism**. A base servo's shaft rotation is not
automatically the same as the model's base angle; linkage leverage, mounting
orientation, and clone hardware can change the relation.

### Use the supplied one-servo commissioning sketch

`arduino/ServoCommissioning/ServoCommissioning.ino` provides small, explicit
pulse adjustments without needing a calibrated synchronization profile. It
controls **one servo at a time** and uses a different, human-readable serial
interface from the synchronization firmware.

1. Turn servo power off, support the mechanism in a fixture, and disconnect the
   other three servos from motor power. If the servo/horn relation is unknown,
   remove or decouple that horn/linkage first. Choose a starting pulse from your
   servo documentation or known working controller; do not guess a full travel
   range from the example values.
2. Open the commissioning sketch in Arduino IDE, select your Uno/Nano and COM
   port, then verify and upload it. Open Serial Monitor at **115200 baud** with
   the line ending set to **Newline**. It starts detached.
3. Connect the chosen servo signal to its assigned pin. Turn on its external
   servo supply when the supported mechanism is clear. Send an explicit ARM
   command with that pin and your verified starting pulse. For example,
   `ARM 11 1500` selects D11 and 1500 µs. **1500 µs is an illustration, not a
   promise of a safe starting pose.** Initial attachment applies that pulse
   immediately, so an unknown loaded linkage must remain disconnected.
4. Send `NUDGE 5` or `NUDGE -5` to change the target by exactly 5 µs. The output
   slews at at most 100 µs/s. Let it settle, measure the model angle with the
   mechanism supported, and send `STATE` to read `APPLIED_US` and `TARGET_US`.
   Record an anchor only when the command has settled and the physical link is
   stationary. STATE remains command data, not a sensor reading.
5. Send `KEEP` at intervals shorter than 5 seconds while taking a measurement.
   If no ARM, NUDGE, or KEEP arrives for 5 seconds, the sketch detaches. **STATE
   does not refresh that timer.** Use a fixture so a timeout cannot drop an
   unsupported link. After a timeout, check the physical position before sending
   ARM again; there is no automatic recovery or known-angle measurement.
6. Send `DISARM` before changing channels, switching power off, or adjusting a
   horn. Repeat for each servo. Initially determine the horn reference without
   its linkage, then verify the installed linkage over a small clear range.

The commissioning hard envelope is 900–2100 µs, and it refuses larger NUDGE
steps. Those checks do not discover mechanical limits. Stop before binding or
persistent buzzing, and do not accumulate repeated nudges toward an unknown
stop. When measurements are complete, close Serial Monitor, turn servo power
off, export the profile, and upload **MeArmSync** in place of this helper.

Edit the JSON in a text editor. Keep the four entries in this exact order:
`yaw`, `shoulder`, `forearm`, `grip`. Field meanings are:

| Field | Meaning |
| --- | --- |
| `version` | Keep `1`. |
| `calibrated` | Keep `false` while preparing the profile; change to `true` only after measuring it. |
| `baudrate` | Keep `115200`, matching the sketch. |
| `watchdog_ms` | Default `500`; allowed `200`–`2000` ms. |
| `name` | The required joint name, in the order above. |
| `pin` | Unique Uno/Nano digital output, D2–D13; D0/D1 are reserved for serial. |
| `model_deg` | Measured angles in strictly increasing order, such as three lower/middle/upper angles. |
| `pulse_us` | Integer pulses corresponding to those same angles, in matching order; strictly increasing or strictly decreasing. |
| `home_deg` | Supported initial angle inside that joint's measured window. |
| `max_us_per_second` | Maximum pulse change per second; default `250`, allowed greater than 0 and at most `1000`. |

Pulse anchors must stay inside the bridge's additional 900–2100 µs envelope.
This software envelope is not a guarantee that the mechanism can travel that
far; use your smaller measured limits. There are no separate `min`/`max` fields:
the exporter takes the smallest and largest measured pulse anchors as firmware
limits. The `--check` command prints the resulting ranges and initial pulses
without opening a port. JSON comments are not supported.

### Model angle conventions

| Calibration joint | Definition | Geometric neutral |
| --- | --- | --- |
| `yaw` | Base rotation; positive turns left | 0° |
| `shoulder` | Upper-arm elevation measured from horizontal | 90° |
| `forearm` | Forearm elevation measured from horizontal | 0° |
| `grip` | One simulated jaw's opening angle | 0° closed |

The shoulder and forearm values are both **absolute elevations**. Forearm is not
the angle between the upper arm and the forearm. The native MuJoCo Control panel
uses radians and represents shoulder as elevation minus 90°. For example,
`joints 0 85 0 5` corresponds to slider targets approximately
`0 -0.08727 0 0.08727`. The bridge performs this conversion.
The default arm scene permits grip up to 28.65°; the cube scene's simulated pad
profile permits up to 40.1°. Your real claw's narrower measured range takes
precedence in either scene.

The official MeArm constructor suggests these nominal servo-degree relationships
for its default calibration:

```text
base servo       = 90 + 2 × model yaw
shoulder servo   = 166 − (16/15) × shoulder elevation
elbow servo      = 90 + 1.2 × forearm elevation
```

These are reference relationships derived from the
[official constructor defaults](https://github.com/MeArm/MeArm-Arduino/blob/master/MeArm.h),
not safe limits or measurements of your kit. The library converts nominal servo
degrees to pulses, including a rounding offset. Copying nominal degrees into a
microsecond field is incorrect. The model's jaw angle also differs from the
library's claw coordinate, so calibrate that channel separately.

### Check, export, and upload

```bat
python scripts/calibrate_hardware.py --check config/hardware_calibration.json
python scripts/calibrate_hardware.py --export-firmware config/hardware_calibration.json
```

The exporter writes `arduino/MeArmSync/calibration.h`. Its calibration identifier
must match the host's JSON. After any calibration, pin, initial-pose, or limit
change, export again and upload the sketch again; editing only the JSON is not
enough.

With the servo supply off, open
`arduino/MeArmSync/MeArmSync.ino` in Arduino IDE. Select the correct board and COM
port, click **Verify**, then **Upload**. For a classic Nano clone, choose the
processor/bootloader variant matching that board if upload fails. The
[Arduino board and port guide](https://support.arduino.cc/hc/en-us/articles/4406856349970-Select-board-and-port-in-Arduino-IDE)
explains board selection. Close Serial Monitor and Serial Plotter before opening
the Python bridge.

## 4. Connect without moving, then enable deliberately

List the available ports:

```bat
python scripts/sync_arduino.py --list-ports
```

Compare that list with Arduino IDE or Windows Device Manager. Replace `COM3`
below with your board's actual port:

```bat
python scripts/sync_arduino.py --port COM3 --calibration config/hardware_calibration.json
```

Connecting begins **disarmed**. Opening USB serial may reset an Uno/Nano; the
firmware starts with all servos detached and does not move to a home pose.
Check the reported protocol, calibration identifier, source mode, and status.
Keep the servo supply off until these checks pass.

The simulator starts at the calibration's `home_deg` pose; the terminal prints
those four values. Support and physically align the real arm to that pose with
motor power off, then turn on servo power. Type `arm` in the Python terminal
when the workspace is clear. Arming resets any changed simulator targets to
calibration HOME, preloads the initial pulses, and attaches the four outputs.
**A servo may move immediately
if its actual position does not match those pulses.** There is no encoder to
verify your alignment.

Use **Control** in the viewer for the four actuator sliders, or type commands in
the terminal. Start with one small change inside your calibrated window:

```text
status
yaw 2
yaw 0
shoulder 88
shoulder 90
forearm 2
forearm 0
grip 5
joints 0 90 0 5
```

These values are examples only. Use values inside your measured window and
substitute your calibrated initial pose; neutral 0/90/0/0 is not universally a
safe startup pose. If the arm turns the wrong way, moves the wrong joint, or
approaches a stop, stop and correct wiring/calibration before continuing.
The general terminal command `home` means geometric `0 90 0 0`, which can differ
from your calibrated HOME; use the printed `joints` values for the latter.

After each joint is verified, try a short, clear Cartesian move:

```text
xyz 145 0 135
status
```

The target must pass both the model's IK limits and your narrower hardware
calibration limits. XYZ moves command the tool endpoint; they do not provide
collision planning. The bridge limits pulse change, so the real arm can take
longer than the simulator to reach a distant target.

Use `disarm` to detach outputs, `estop` to detach and latch the firmware stop,
and `quit` to exit. Support the arm before detaching. Reset the board to clear a
latched stop, check the cause, reconnect, realign, and arm explicitly again.
Keep the supply switch available if software or USB is unresponsive.

## 5. Arduino commands driving the simulation

Use this mode when an Arduino joystick, potentiometer panel, or Arduino program
owns the servo targets. It requires **LOCAL** firmware; the reader rejects a
HOST-controlled board. Choose either the ready four-potentiometer example below
or your own local controller hook.

### Four potentiometers or two joysticks

The provided `arduino/MeArmSync/local_joystick.example.h` reads four analog inputs
and uses a held enable button. With power off, wire four approximately 10 kΩ
potentiometers, or two two-axis joystick modules, as follows:

| Input | Connection |
| --- | --- |
| Yaw wiper / first axis | A0 |
| Shoulder wiper / second axis | A1 |
| Forearm wiper / third axis | A2 |
| Grip wiper / fourth axis | A3 |
| Each potentiometer's outer terminals | Arduino 5 V and GND |
| Normally open enable button | D2 to GND; the firmware uses an internal pull-up. |

Keep D2 free of servo outputs; the normal D11/D10/D9/D6 mapping does so. These
input potentiometers command the arm; they do not sense its actual joint angles.

Add both lines above `#include "calibration.h"` in `MeArmSync.ino`, then verify and
upload the sketch with your generated calibration header:

```cpp
#define MEARM_SOURCE_LOCAL true
#define MEARM_LOCAL_JOYSTICKS
```

Start with narrow measured limits and the arm supported at calibration HOME.
The example maps each analog input across that servo's **pulse** window; the
middle input value need not equal its calibrated HOME. Set inputs near the
corresponding initial pulses before enabling. If a knob's direction is
inconvenient, reverse its two outer terminals with power off or adapt the input
mapping. Confirm each axis individually.

Release the enable button after boot, then hold it to attach at HOME. While held,
the firmware samples the four inputs every 20 ms and slews toward their targets.
Release the button to detach after its approximately 30 ms debounce interval.
The supported arm may move as soon as you hold enable, and may fall when you
release it. If a timeout occurs, continuing to hold the button does not rearm:
release it, check the mechanism, and hold again. A latched ESTOP requires a board
reset.

### Your own Arduino input logic

For a different controller, define only LOCAL mode above the calibration include:

```cpp
#define MEARM_SOURCE_LOCAL true
```

Implement the existing `updateLocalController()` hook with your joystick/button
logic. It runs on every firmware loop and must return promptly:

- On an explicit physical enable button press, call `armLocalController()`.
  It attaches at the calibrated initial pulses and returns whether it succeeded.
- While enabled, call `setLocalTargets(pulses)` regularly with an `int pulses[4]`
  in yaw/shoulder/forearm/grip order. Every value must be inside its measured
  pulse window. The function validates the complete command, refreshes the
  watchdog, and returns whether it accepted the target.
- On a local stop input, call `disarmLocalController()`.

Without the example flag, the default hook is deliberately empty and never arms
on boot. Choose unused inputs suitable for your existing controller. Preserve
its pulse checks, slew limiter, and stop handling.
Do not let a second `MeArm`/`Servo` object write the same output pins: the reported
state would no longer describe the output. Do not block the hook with long
`delay()` calls or the original library's blocking movement routines.

### Start the read-only observer

With the LOCAL sketch uploaded, Serial Monitor closed, and the arm supported:

```bat
python scripts/sync_arduino.py --port COM3 --calibration config/hardware_calibration.json --direction hardware-to-sim
```

The reader does not arm or send movement targets. Use your physical local enable
control. The reader converts reported applied servo pulses back to model angles
and writes MuJoCo actuator controls; MuJoCo moves the linkage under physics.
Dragging the real arm with power off will not update the simulation. Closing the
observer leaves the LOCAL controller responsible for motion and stopping.

This reader cannot listen to an arbitrary existing sketch that prints unrelated
text. That sketch must implement this protocol or be adapted to publish its
applied commands. Also, one Windows process owns the COM port, so you cannot
run a HOST bridge and a separate reader against the same USB port simultaneously.

## 6. Coordinates and the official MeArm library

The project uses X forward, Y left, and Z up from the floor. The official Arduino
library uses X right, Y forward, and Z measured from shoulder height. With this
model's 55 mm shoulder height, convert Arduino library millimetres to simulation
millimetres as follows:

```text
X_sim =  Y_Arduino
Y_sim = −X_Arduino
Z_sim =  Z_Arduino + 55
```

For example, library `(0,148,80)` corresponds to simulation `(148,0,135)`.
Measure the actual shoulder height and any physical table/base offset when
registering your hardware. The
[official geometry description](https://github.com/MeArm/MeArm-Arduino/blob/master/Geometry.md)
defines the Arduino origin.

The current official `config.h` defaults to V3 `L3=22 mm` and comments V0.3–V1.2
`L3=68 mm`; this Classic V1 simulation uses 68 mm combined horizontal offsets and
80 mm upper/forearm lengths. If you use the original library's Cartesian IK on
your real V1, configure its V1 dimensions. The direct joint/pulse bridge does not
call that library's IK.
[Official configuration source](https://github.com/MeArm/MeArm-Arduino/blob/master/config.h).

The new sketch uses Arduino Servo directly because the official MeArm class has
private servos and no public four-joint setter, while its `begin()` immediately
commands an initial pose and opens the claw. Its `getX/Y/Z()` values are stored
command coordinates. The bridge preserves the official geometry conventions
without relying on that startup motion or treating those values as sensors.
[Official implementation](https://github.com/MeArm/MeArm-Arduino/blob/master/MeArm.cpp).

## 7. Timing, stops, and what to verify

The host sends commands at 20 Hz by default; `--hz` allows 5–50 Hz subject to at
least three updates per watchdog interval. The Arduino applies an independent
per-channel pulse slew limit. The default profile rate limit is 250 µs/s and
the communication watchdog is 500 ms. A missing heartbeat, disarm, or normal
HOST exit detaches servo outputs. Lost communication requires operator recovery;
the bridge does not reconnect and resume motion automatically. The checksum
detects accidental frame corruption, not malicious traffic.
In LOCAL mode, your controller refreshes the watchdog when it submits targets;
closing a read-only observer does not stop an otherwise active local controller.

Before expanding the calibrated window, verify:

- Each of the four small positive joint commands moves the corresponding real
  link in the same direction as the simulation.
- Reference and nearby poses agree when measured with an angle gauge; visually
  similar motion alone does not validate the mapping.
- The Arduino's reported pulses match the expected calibration and change no
  faster than its slew limit. This validates transmitted commands, not physical
  tracking accuracy.
- Disarm and watchdog behavior work with the arm supported and no payload.
- Supply voltage stays within the servo specification under load, with no board
  resets or persistent motor buzzing.

Only then expand ranges incrementally and re-export/re-upload calibration.
Maintain extra clearance for servo backlash and flex. Joint limits are not a
collision model: a pose valid per joint can still bind an assembled linkage.

The colored cubes and tabletop in the simulation have no automatic physical
registration. A virtual cube's color/position is not a camera detection or a
measurement of a real cube. The simulation pick-and-place program must not be
assumed safe to replay on the arm: first establish a measured table frame,
matching object poses, claw gap, clearance, payload capability, and a supervised
physical approach. This synchronization runner provides manual/XYZ control; it
does not automatically execute the simulated cube task on hardware.
You may add `--scene cubes` to the synchronization command to use that visual
scene with manual joint/XYZ controls; it does not add physical object sensing
or start a pick-and-place sequence.

### Serial integration details

The supplied host handles framing. You do not need to type protocol messages in
Arduino Serial Monitor. For another controller implementation, use 115200 baud,
8 data bits, no parity, one stop bit, and no flow control. Each ASCII payload is
followed by `*XX` and a newline, where `XX` is the uppercase two-digit XOR of all
payload bytes. Fields are space-separated. Sequences are unsigned 0–65535 and
responses echo the request sequence.

| Host payload | Firmware response payload |
| --- | --- |
| `HELLO seq` | `READY seq 1 profile_id HOST_or_LOCAL armed` |
| `STATE seq` | `STATE seq armed reason base_us shoulder_us elbow_us grip_us` |
| `ARM seq p0 p1 p2 p3` | `ACK seq ARM`; HOST only, calibrated, not stopped, initially detached. |
| `SET seq p0 p1 p2 p3` | `ACK seq SET`; HOST only, already armed, all pulses inside firmware limits. |
| `KEEP seq` | `ACK seq KEEP`; HOST only, refreshes the communication watchdog when armed. |
| `DISARM seq` | `ACK seq DISARM`; detaches outputs. |
| `ESTOP seq` | `ACK seq ESTOP`; detaches and latches until board reset. |

Rejected messages return `ERROR seq reason`. The firmware checks framing,
checksum, token count, numeric format, bounds, ownership, and arm/stop state.
The host checks protocol version and calibration ID before enabling movement.
`STATE` pulses are the slew-limited commands currently applied, or their last
values if detached. `armed=0` means those numbers are no longer active signals.
The calibration ID identifies the normalized JSON, including its confirmation
flag; it is a mismatch check, not cryptographic authentication. Do not add free
text debug prints to the protocol stream during operation.

## 8. Troubleshooting

| Symptom | Check |
| --- | --- |
| `No module named serial` | Activate `mearm`, then run `python -m pip install -r requirements-hardware.txt`. The package is `pyserial`, not the unrelated package named `serial`. |
| No COM port | Use a USB data cable, check Device Manager, reconnect the board, and install the USB interface driver supplied by your board manufacturer if needed. A charging-only cable cannot carry serial data. |
| Access denied / port busy | Close Arduino Serial Monitor/Plotter, VS Code serial extensions, other Python bridge windows, and other programs holding the port. |
| Calibration mismatch | Export the same JSON used by Python and upload its generated header with the sketch. A board reset does not update its compiled calibration. |
| Refuses to arm | Check calibration confirmation, source mode, initial pose, firmware stop latch, and the status/error text. Do not widen bounds simply to silence an error. |
| Out-of-range joint | Keep commands/sliders inside your hardware window. A model-wide slider range can be wider than the real arm's allowed calibration. |
| Wrong direction or offset | Check servo identity, horn mounting, measured middle angle, and whether the pulse mapping increases or decreases. Recheck both absolute link elevations. |
| Jitter, resets, disconnects | Check external supply current, motor wiring, shared ground, cable quality, and load. Arduino's [servo troubleshooting guide](https://support.arduino.cc/hc/en-us/articles/360017053760-Troubleshoot-servo-motors) covers power and pin errors. |
| Simulation follows but real joint stalls | Stop; inspect power, mechanical interference, horn security, and load. Command telemetry cannot detect a stall. |
| Hardware-to-sim is stationary | It displays command telemetry, so provide LOCAL targets through the hook. It cannot sense manual back-driving. |
| Stops while moving | Read the fault, support/disarm the arm, and investigate timing/reset/power. Do not increase the watchdog before resolving the cause. |

Port enumeration and explicit serial timeouts are described in the
[pySerial introduction](https://pyserial.readthedocs.io/en/latest/shortintro.html).

## 9. Adding actual joint feedback later

For measured synchronization, add output-side encoders or feedback-capable
servos and publish **measured** angles separately from command targets. Calibrate
sensor zeros and signs in these same geometric coordinates, timestamp samples,
check stale data, and compare measured versus requested motion. Joint feedback
can identify tracking errors; it still does not localize cubes or prove a grasp.
Vision or other object sensing is a separate addition.

Software checks and MuJoCo tests do not replace a physical calibration. The
supplied bridge has not measured or validated your particular arm's wiring,
servo travel, supply, or real-world tracking accuracy.
