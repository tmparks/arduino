# arduino

1.  Download and install [arduino-cli](https://docs.arduino.cc/arduino-cli/installation/#download)

2.  Install libraries and platforms
    ```
    arduino-cli config init
    arduino-cli config set library.enable_unsafe_install true 
    arduino-cli lib install --git-url https://github.com/MarkusLange/Bosch-BSEC2-Library.git
    arduino-cli config set library.enable_unsafe_install false
    arduino-cli config set board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
    arduino-cli core update-index
    arduino-cli core install teensy:avr
    cp platform.txt {directories.data}/packages/teensy/hardware/avr/*/platform.txt
    ```
    * Replace `{directories.data}` with ...
      (see [Open the Arduino15 folder](
        https://support.arduino.cc/hc/en-us/articles/360018448279-Open-the-Arduino15-folder)
      and [Default directories](
        https://docs.arduino.cc/arduino-cli/configuration/#default-directories))
      * Windows: `C:\Users\{username}\AppData\Local\Arduino15`
      * macOS: `/Users/{username}/Library/Arduino15`
      * Linux: `/home/{username}/.arduino15`
      * Replace `{username}` with your user name

3.  Configure sketch (edit `AirQuality/sketch.yaml`)
    * Replace `{directories.user}` with ...
      (see [Sketch project file](
        https://docs.arduino.cc/arduino-cli/sketch-project-file/)
      and [Default directories](
        https://docs.arduino.cc/arduino-cli/configuration/#default-directories))
      * Windows: `C:\Users\{username}\Documents\Arduino`
      * macOS: `/Users/{username}/Documents/Arduino`
      * Linux: `/home/{username}/Arduino`
      * Replace `{username}` with your user name

4.  Compile
    ```
    arduino-cli compile <sketch>
    ```
    * Replace `<sketch>` with the name of the sketch (e.g. AirQuality or Blink)

5.  Upload
    ```
    arduino-cli board list
    arduino-cli upload --port <port> <sketch>
    ```
    * Replace `<port>` with the port shown for `teensy:avr:teensy41`
    * Replace `<sketch>` with the name of the sketch (e.g. AirQuality or Blink)

## AirQuality

## Blink

Blinks the built-in LED on the Teensy board.
