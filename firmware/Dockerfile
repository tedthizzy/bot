# Build image for firmware/build.sh: Debian + arduino-cli.
#
# Neither hub.docker.com/r/arduino/arduino-cli nor ghcr.io/arduino/arduino-cli
# was pullable when this was written, so the image is built here. It runs
# natively on arm64 and amd64; every esp32:esp32@2.0.17 tool has an
# aarch64-linux-gnu build. python3 is needed by the core's gen_esp32part.py and
# python3-serial by its esptool (a Python bundle on Linux, not a binary).
FROM debian:bookworm-slim

ARG ARDUINO_CLI_VERSION=1.5.1

RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl python3 python3-serial \
 && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | BINDIR=/usr/local/bin sh -s "${ARDUINO_CLI_VERSION}" \
 && arduino-cli version

# The core, the downloads and the libraries live on a bind mount (firmware/.cache)
# so a second build does not download 600 MB again.
ENV ARDUINO_DIRECTORIES_DATA=/cache/data \
    ARDUINO_DIRECTORIES_DOWNLOADS=/cache/downloads \
    ARDUINO_DIRECTORIES_USER=/cache/user \
    HOME=/cache/home

WORKDIR /work
ENTRYPOINT ["arduino-cli"]
