#!/bin/sh -e
# Setup arduino-cli environment.
# This script is idempotent and will do no harm if run multiple times.

# https://docs.arduino.cc/arduino-cli/installation/#download
# https://docs.arduino.cc/arduino-cli/configuration/#default-directories
# https://docs.arduino.cc/arduino-cli/sketch-project-file/
# https://support.arduino.cc/hc/en-us/articles/360018448279-Open-the-Arduino15-folder

HERE="$(realpath $(dirname "$0"))" # location of this script
TEENSY_VERSION=1.59.0
BSEC2_VERSION=1.6.2400

# Exit if arduino-cli command is not found.
arduino-cli version || exit 1

if [ $(uname -s) = Darwin ] # macOS
then
    SED="sed -i .orig"
else # Linux
    SED="sed --in-place=.orig"
    sudo wget --directory-prefix=/etc/udev/rules.d/ https://www.pjrc.com/teensy/00-teensy.rules
fi

arduino-cli config init --overwrite
arduino-cli config set network.connection_timeout 60m0s
arduino-cli config set library.enable_unsafe_install true 
arduino-cli lib install --git-url https://github.com/MarkusLange/Bosch-BSEC2-Library.git
arduino-cli config set library.enable_unsafe_install false
arduino-cli config set board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core update-index
arduino-cli core install teensy:avr@$TEENSY_VERSION

ARDUINO_DIRECTORIES_DATA="$(arduino-cli config get directories.data)"
ARDUINO_DIRECTORIES_USER="$(arduino-cli config get directories.user)"

# Global platform configuration.
mkdir -p "$ARDUINO_DIRECTORIES_USER/hardware/"
cp "$HERE/platform.txt" "$ARDUINO_DIRECTORIES_USER/hardware/"

# Modify sketch configuration.
# Note: % delimiter allows / in replacement.
FILE="$HERE/AirQuality/sketch.yaml"
EXPRESSION="{directories.user}\(/libraries/\)"
REPLACEMENT="$ARDUINO_DIRECTORIES_USER\\1"
$SED s%"$EXPRESSION"%"$REPLACEMENT"% "$FILE"
