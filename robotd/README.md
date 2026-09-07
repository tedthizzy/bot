# robotd — the only process that can move the robot

Owns the serial link to the motor controller and every motion safety decision
on the host. Validates each skill request against the catalog bounds, turns an
accepted one into a velocity profile, streams it at the command rate, tracks
odometry, publishes state, and cancels on any of its timeouts.

Must not import a model, audio or camera library, and must not wait on the
network for anything in the motion path. It is the process that must not crash,
so everything that can crash lives elsewhere.

A request is refused unless it is a known skill, inside bounds, from an allowed
source, newly sequenced, inside its validity window, within the instruction's
cumulative motion budget, and currently permitted by the controller's state.
