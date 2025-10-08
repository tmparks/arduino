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
        /pkg/arduino-cli.deb \
    && rm --recursive --force /var/lib/apt/lists/*

# Become unprivileged user
USER ubuntu
WORKDIR /home/ubuntu

RUN arduino-cli config init \
    && arduino-cli config set board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json \
    && arduino-cli core update-index \
    && arduino-cli core install teensy:avr
