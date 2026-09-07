# brain — voice, camera, model and the interaction state machine

Hears a command, decides what to ask the model, validates what comes back, and
sends a skill request to robotd over the local WebSocket.

Never writes to the serial port. Never holds a motion goal; robotd owns that.
A model response that arrives after a stop, after a newer instruction, or with
an unknown skill name cannot move the robot.

Audio, camera, speech recognition and speech synthesis sit behind adapters so
the same code runs on a Mac with typed input and a stub voice, and on the Pi
with the real microphone, camera and speakers.
