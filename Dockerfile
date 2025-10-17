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
        ssh-client \
        sudo \
        wget \
        /pkg/arduino-cli.deb \
    && rm --recursive --force /var/lib/apt/lists/*

RUN echo "ubuntu ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/nopasswd

# Become unprivileged user
USER ubuntu
WORKDIR /home/ubuntu
