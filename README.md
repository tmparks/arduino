# arduino

0.  Delete (or rename) the following directories (if they exist) to remove
    any outdated components of the development environment
    * Windows Command Prompt:
      ```
      rmdir /s %LOCALAPPDATA%\Arduino15
      rmdir /s %USERPROFILE%\Documents\Arduino
      ```
    * macOS terminal:
      ```
      rm -rf ~/Library/Arduino15
      rm -rf ~/Documents/Arduino
      ```
    * Linux terminal:
      ```
      rm --recursive --force ~/.arduino15
      rm --recursive --force ~/Arduino
      ```

1.  Download and install [arduino-cli](https://docs.arduino.cc/arduino-cli/installation/#download)
    (downloads less than 20 MB)
    * Windows: download and launch msi installer
    * macOS or Linux: download and extract files from archive
      * Replace `<dir>` below with the directory containing the extracted files
      ```
      cd <dir>
      sudo mv arduino-cli /usr/local/bin
      ```

2.  Download and extract [code](https://github.com/tmparks/arduino/archive/refs/heads/main.zip)
    (downloads less than 1 MB)

3.  Setup development environment
    (downloads less than 150 MB)
    * Replace `<dir>` below with the directory containing the extracted files
      from the previous step
    * Windows Command Prompt
      ```
      chdir <dir>
      setup.bat
      ```
    * macOS or Linux terminal
      ```
      cd <dir>
      ./setup.sh
      ```

4.  Compile
    (downloads less than 1 MB, first time only)
    * Replace `<sketch>` below with the name of the sketch (e.g. AirQuality or Blink)
    ```
    arduino-cli compile <sketch>
    ```

5.  Upload
    * Replace `<port>` below with the port shown for the `teensy:avr:teensy41` board
    * Replace `<sketch>` below with the name of the sketch (e.g. AirQuality or Blink)
    ```
    arduino-cli board list
    arduino-cli upload --port <port> <sketch>
    ```
