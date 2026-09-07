# tests

Unit tests run anywhere. The gate scripts run against the simulator and answer
one question each: can the robot move when it must not?

The fault injection gate is the important one. It sends malformed and
out-of-bounds requests, replays a request id, delivers a model response after a
stop, kills and suspends each service, pulls the serial link, and takes the
model endpoint away, asserting no motion in every case.

Two gates need hardware and are written as checklists for a person to run.
