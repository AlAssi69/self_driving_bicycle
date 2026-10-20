# Arduino serial protocol

The `bicycle_hardware` plugin and the Arduino firmware talk over USB serial with short ASCII
lines. [`protocol.hpp`](../bicycle_hardware/firmware/bicycle_uno/protocol.hpp) defines it
once for both sides; this page explains it.

- 115200 baud, 8N1. Each line ends with `\n`; a `\r` before it is ignored.
- Fields are separated by single spaces; numbers are decimal integers.
- Lines are at most 63 characters; longer ones are rejected.
- The protocol is versioned. The plugin refuses to activate when the firmware reports a
  different name or version: increase `kVersion` on every incompatible change.

## Requests (PC → Arduino)

| Request | Reply | Meaning |
|---|---|---|
| `V` | `V bicycle_uno <version>` | Firmware name and protocol version. |
| `L <max_steps> <steps_s> <steps_s2>` | `K <max_steps> <steps_s> <steps_s2>` | Steering limits: angle (steps), speed (steps/s) and acceleration (steps/s²). The reply holds the values in effect after capping them at the firmware's hard limits. |
| `C <pwm> <steering_steps>` | none | Drive PWM (−255…255, the sign is the direction) and steering target. Never answered, so the plugin does not wait for it. |
| `Q <seq>` | `S <seq> <counts> <period_us> <steering_steps> <flags>` | State query; `seq` (0…65535) is echoed so late replies can be dropped. |
| `Z` | `K` | Declares the current steering position as straight ahead. |

An unknown request gets `E 1`, malformed arguments `E 2` and an overlong line `E 3`. A
malformed `C` gets no reply; it sets the command-error flag instead.

## State reply

`S <seq> <counts> <period_us> <steering_steps> <flags>`

| Field | Meaning |
|---|---|
| `counts` | Signed encoder pulse count since the Arduino started. The single-channel sensor cannot sense direction: each pulse takes the sign of the drive command at that moment. |
| `period_us` | Signed time between the last two pulses or, when longer, since the last pulse, so the speed estimate decays when the wheel stops. 0 until two pulses were seen. |
| `steering_steps` | AccelStepper's current position. |
| `flags` | Bit 0: the watchdog stopped the drive. Bit 1: a steering target was clamped. Bit 2: the stepper driver is enabled. Bit 3: a command was rejected. Bits 1 and 3 are cleared once reported. |

## Cycle

Every control cycle (100 Hz) the plugin sends `Q <seq>`, waits up to `read_timeout_ms` for
the matching `S` reply (`read()`), then sends `C <pwm> <steps>` (`write()`). At start-up it
sends `V` until the Arduino answers (opening the port resets the Uno), then `L`.

## Safety

- **Watchdog.** The firmware sets the PWM to 0 when no `C` request arrives for 300 ms
  (`kCommandTimeoutMs`). The steering holds its position.
- **Hard limits.** `config.hpp` caps the steering angle, speed and acceleration and the PWM.
  `L` can only tighten them.
- **Link loss.** After `max_consecutive_errors` failed exchanges the plugin reports an error;
  the controller manager then stops the controllers and the plugin sends a last stop command.
