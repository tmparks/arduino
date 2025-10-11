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
    ```

3.  Modify platform configuration
    (see [Open the Arduino15 folder](
      https://support.arduino.cc/hc/en-us/articles/360018448279-Open-the-Arduino15-folder)
    and [Default directories](
      https://docs.arduino.cc/arduino-cli/configuration/#default-directories))
    * Windows PowerShell
      ```
      copy platform.txt $Env:LOCALAPPDATA\Arduino15\packages\teensy\hardware\avr\1.59.0\
      ```
    * Windows Command Prompt
      ```
      copy platform.txt %LOCALAPPDATA%\Arduino15\packages\teensy\hardware\avr\1.59.0\
      ```
    * macOS
      ```
      cp platform.txt $HOME/Library/Arduino15/packages/teensy/hardware/avr/*/
      ```
    * Linux
      ```
      cp platform.txt $HOME/.arduino15/packages/teensy/hardware/avr/*/
      ```

4.  Modify sketch configuration (edit `AirQuality/sketch.yaml`)
    * Replace `{directories.user}` with ...
      (see [Sketch project file](
        https://docs.arduino.cc/arduino-cli/sketch-project-file/)
      and [Default directories](
        https://docs.arduino.cc/arduino-cli/configuration/#default-directories))
      * Windows: `C:/Users/{username}/Documents/Arduino`
      * macOS: `/Users/{username}/Documents/Arduino`
      * Linux: `/home/{username}/Arduino`
    * Replace `{username}` with your user name

5.  Compile
    ```
    arduino-cli compile <sketch>
    ```
    * Replace `<sketch>` with the name of the sketch (e.g. AirQuality or Blink)

6.  Upload
    ```
    arduino-cli board list
    arduino-cli upload --port <port> <sketch>
    ```
    * Replace `<port>` with the port shown for `teensy:avr:teensy41`
    * Replace `<sketch>` with the name of the sketch (e.g. AirQuality or Blink)

## AirQuality

## Blink

Blinks the built-in LED on the Teensy board.
