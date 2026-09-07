# bot

A rover project exploring voice, vision, and controlled movement using existing hardware.

This repository contains the proposed structure. Firmware, services, simulation, and deployment are currently placeholders.

## Goals

- Understand spoken requests, describe scenes, and find objects with a camera.
- Translate requests into a small catalog of skills with explicit movement limits.
- Stop motion when commands expire or communication fails.
- Develop and test on a Mac, then validate the system on the physical robot.

## Why this path

The proposed structure reuses a Raspberry Pi 4 to coordinate speech, camera input, and the user interface. A separate GPU box would run the vision-language model, keeping that workload off the Pi.

An ESP32-S3 would handle motor control, hard limits, and watchdogs. This separates motor timing from slow or failed model calls. The model would propose skills, with deterministic validation between its output and motion. A shared skill catalog would keep bounds consistent across services.

The proposed structure also shares a portable firmware core between the controller and a Mac simulator. That would let us test the real protocol parser, watchdog, and control logic before bench and chassis validation.

See [PLAN.md](PLAN.md) for progress and [docs/adr/](docs/adr/) for the planned decision records.
