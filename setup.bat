@echo off
rem Setup arduino-cli environment.
rem This script is idempotent and will do no harm if run multiple times.

rem https://docs.arduino.cc/arduino-cli/installation/#download
rem https://docs.arduino.cc/arduino-cli/configuration/#default-directories
rem https://docs.arduino.cc/arduino-cli/sketch-project-file/
rem https://support.arduino.cc/hc/en-us/articles/360018448279-Open-the-Arduino15-folder

set HERE=%~dp0
set TEENSY_VERSION=1.59.0
set BSEC2_VERSION=1.6.2400

arduino-cli config init --overwrite
arduino-cli config set library.enable_unsafe_install true 
arduino-cli lib install --git-url https://github.com/MarkusLange/Bosch-BSEC2-Library.git
arduino-cli config set library.enable_unsafe_install false
arduino-cli config set board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core update-index
arduino-cli core install teensy:avr@%TEENSY_VERSION%

set ARDUINO_DIRECTORIES_DATA=%LOCALAPPDATA%\Arduino15
set ARDUINO_DIRECTORIES_USER=%USERPROFILE%\Documents\Arduino

rem Global platform configuration.
mkdir "%ARDUINO_DIRECTORIES_USER%\hardware\"
copy "%HERE%platform.txt" "%ARDUINO_DIRECTORIES_USER%\hardware\"

rem Modify sketch configuration.
set FILE='%HERE%AirQuality/sketch.yaml'
set EXPRESSION='{directories.user}(/libraries/)'
set REPLACEMENT='%ARDUINO_DIRECTORIES_USER%$1'
powershell -Command "& {(Get-Content %FILE%) -replace %EXPRESSION%, %REPLACEMENT% | Set-Content %FILE%}"
