# https://docs.docker.com/build/building/best-practices/

FROM scratch AS pkg

ARG ARCH=arm64
ARG VERSION=1.3.1
ADD https://github.com/arduino/arduino-cli/releases/download/v$VERSION/arduino-cli_$VERSION-1_$ARCH.deb \
    /arduino-cli.deb

FROM ubuntu

RUN --mount=from=pkg,target=/pkg \
    apt-get update \
    && apt-get install --yes --no-install-recommends \
        ca-certificates \
        git \
        /pkg/arduino-cli.deb \
    && rm --recursive --force /var/lib/apt/lists/*

# Become unprivileged user
USER ubuntu
WORKDIR /home/ubuntu

RUN arduino-cli config init \
    && arduino-cli config set board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json \
    && arduino-cli core update-index \
    && arduino-cli core install teensy:avr \
    && arduino-cli lib install \
        "Adafruit PM25 AQI Sensor" \
        "DFRobot_C4001" \
    && arduino-cli config set library.enable_unsafe_install true \
    && arduino-cli lib install --git-url \
        https://github.com/MarkusLange/Bosch-BSEC2-Library.git \
        https://github.com/boschsensortec/Bosch-BME68x-Library.git \
    && arduino-cli config set library.enable_unsafe_install false

COPY platform.txt .arduino15/packages/teensy/hardware/avr/1.59.0/
