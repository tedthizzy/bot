# firmware — the motor and safety controller

Two pieces, deliberately separated.

`core/` is freestanding C with no vendor headers: the frame codec, the session
and arm state machine, the command timeout, the speed and acceleration caps,
the wheel control loop and the fault latch. It compiles on a Mac and is covered
by host tests, so most of the safety behaviour is provable before any hardware
exists.

`esp32/` is the ESP-IDF application that binds that core to real peripherals:
encoders, motor driver, distance sensors, bumper, emergency stop sense, and the
battery measurement.

The controller boots with motors off and stays that way until the host completes
a handshake and an explicit arm. It refuses any speed above its compiled caps,
and no message in the protocol can raise one.
