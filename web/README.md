# web — face, teleop and the phone head

Serves the robot's face, a teleop pad, and the endpoints a phone browser uses
as a removable head. Talks to robotd over the same WebSocket as everything else
and gets no privileges for it.

Never touches the serial port. Browser and phone requests carry their own source
label and pass the same validator as the model's, so the rover behaves
identically whether or not any of this is running.
