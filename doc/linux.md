# Linux setup

1.  Delete (or rename) the following directories (if they exist)
    to remove any outdated components of the development environment.
    ```
    rm --recursive --force ~/.arduino15
    rm --recursive --force ~/Arduino
    ```

2.  Download the 64-bit Linux archive
    for the latest release of [arduino-cli](https://docs.arduino.cc/arduino-cli/installation/#download).
    (downloads less than 20 MB)
    * Extract all files from the archive.
    * Replace `<dir>` below with the directory containing the extracted files.
    ```
    cd <dir>
    sudo mv arduino-cli /usr/local/bin
    ```

3.  Download the [code](https://github.com/tmparks/arduino/archive/refs/heads/main.zip)
    for this project.
    (downloads less than 1 MB)
    * Extract all files from the zip archive.
    * Move the resulting directory to a convenient location.

4.  Setup the development environment.
    (downloads less than 150 MB)
    * Replace `<dir>` below with
      the directory containing the files from the previous step.
    ```
    cd <dir>
    ./setup.sh
    ```

5.  Edit the file AirQuality.ino in the AirQuality directory
    to modify `BOX_NUMBER` and other configuration parameters.

6.  Compile the AirQuality sketch.
    (downloads less than 1 MB, first time only)
    ```
    arduino-cli compile AirQuality
    ```

7.  Upload the AirQuality sketch.
    * Use a USB cable to connect to a teensy board.
    * Replace `<port>` below with the port shown for the teensy board.
    ```
    arduino-cli board list
    arduino-cli upload --port <port> AirQuality
    ```

8.  Monitor messages from the board.
    * Replace `<port>` below with the port for the teensy board.
    ```
    arduino-cli monitor --port <port>
    ```
