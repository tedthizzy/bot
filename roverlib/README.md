# roverlib — shared contracts

The single source of truth for every message that crosses a process boundary.
Every other component imports this and none of them redefine a shape.

Holds the skill catalog with its bounds, the pydantic models for the model's
proposed skill call and for vision answers, the world state sent to the model,
the robotd WebSocket messages, the wire codec for the link to the motor
controller, and the config loader.

Depends on nothing in this repo. Imports no hardware library, no model library
and no audio library, so it stays importable and testable anywhere.
