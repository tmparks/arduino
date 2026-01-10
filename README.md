# arduino

1.  Delete (or rename) the following directories (if they exist) to remove
    any outdated components of the development environment
    * Windows Command Prompt:
      ```
      rmdir /s /q %LOCALAPPDATA%\Arduino15
      rmdir /s /q %USERPROFILE%\Documents\Arduino
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

2.  Download and install [arduino-cli](https://docs.arduino.cc/arduino-cli/installation/#download)
    (downloads less than 20 MB)
    * Windows: download and launch msi installer, then restart your computer
    * macOS or Linux: download and extract files from archive
      * Replace `<dir>` below with the directory containing the extracted files
      ```
      cd <dir>
      sudo mv arduino-cli /usr/local/bin
      ```

3.  Download and extract [code](https://github.com/tmparks/arduino/archive/refs/heads/main.zip)
    (downloads less than 1 MB)
    * Move the resulting directory to a convenient location

4.  Setup development environment
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

5.  Edit AirQuality.ino to modify `BOX_NUMBER` and other configuration parameters.

6.  Compile
    (downloads less than 1 MB, first time only)
    * Replace `<sketch>` below with the name of the sketch (e.g. AirQuality or Blink)
    ```
    arduino-cli compile <sketch>
    ```

7.  Upload
    * Replace `<port>` below with the port shown for the `teensy:avr:teensy41` board
    * Replace `<sketch>` below with the name of the sketch (e.g. AirQuality or Blink)
    ```
    arduino-cli board list
    arduino-cli upload --port <port> <sketch>
    ```

8.  Monitor
    * Replace `<port>` below with the port for the `teensy:avr:teensy41` board
    ```
    arduino-cli monitor --port <port>
    ```

## Plot Air Quality Data

1.  Download and install [python](https://www.python.org/downloads/).

2.  Install libraries.
    ```
    pip install matplotlib
    pip install pandas
    ```

3.  Edit plot.py to modify `BOX_FILES`, `EPAM_FILES`, and other configuration parameters.

4.  Plot data.
    ```
    python3 AirQuality/plot.py
    ```
