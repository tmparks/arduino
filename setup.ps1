# Setup arduino-cli environment.
# This script is idempotent and will do no harm if run multiple times.

# https://docs.arduino.cc/arduino-cli/installation/#download
# https://docs.arduino.cc/arduino-cli/configuration/#default-directories
# https://docs.arduino.cc/arduino-cli/sketch-project-file/
# https://support.arduino.cc/hc/en-us/articles/360018448279-Open-the-Arduino15-folder

$HERE = $PSScriptRoot # location of this script
$TEENSY_VERSION = "1.59.0"
$BSEC2_VERSION = "1.6.2400"

arduino-cli config init --overwrite
arduino-cli config set library.enable_unsafe_install true 
arduino-cli lib install --git-url https://github.com/MarkusLange/Bosch-BSEC2-Library.git
arduino-cli config set library.enable_unsafe_install false
arduino-cli config set board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core update-index
arduino-cli core install teensy:avr@$TEENSY_VERSION

$ARDUINO_DIRECTORIES_DATA = (arduino-cli config get directories.data | Out-String).Trim()
$ARDUINO_DIRECTORIES_USER = (arduino-cli config get directories.user | Out-String).Trim()

# Modify platform configuration to enable precompled libraries.
# Note: trailing space ensures idempotence.
# https://github.com/MarkusLange/Bosch-BSEC2-Library/tree/master#4-modify-the-platformtxt-file
$FILE = $ARDUINO_DIRECTORIES_DATA + "/packages/teensy/hardware/avr/" + $TEENSY_VERSION + "/platform.txt"
$EXPRESSION = "({build.flags.libs})`$"
$REPLACEMENT = "{compiler.libraries.ldflags} `$1 `ncompiler.libraries.ldflags="
(Get-Content $FILE) -replace $EXPRESSION, $REPLACEMENT | Set-Content $FILE

# Modify sketch configuration with user directory.
$FILE = $HERE + "/AirQuality/sketch.yaml"
$EXPRESSION = "{directories.user}(/libraries/)"
$REPLACEMENT = $ARDUINO_DIRECTORIES_USER + "`$1"
(Get-Content $FILE) -replace $EXPRESSION, $REPLACEMENT | Set-Content $FILE
