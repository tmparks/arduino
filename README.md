# arduino

1.  Setup the development environment.
    * [Windows](doc/windows.md)
    * [macOS](doc/macOS.md)
    * [Linux](doc/linux.md)

2.  Edit the AirQuality.ino file in the AirQuality directory
    to modify `BOX_NUMBER` and other configuration parameters.

    ![edit](doc/edit.png)

3.  Compile the AirQuality sketch.
    (downloads less than 1 MB, first time only)
    ```
    arduino-cli compile AirQuality
    ```

    ![compile](doc/compile.png)

4.  Upload the AirQuality sketch.
    * Use a USB cable to connect to a teensy board.
    * Replace `<port>` below with the port shown for the teensy board.
    ```
    arduino-cli board list
    arduino-cli upload --port <port> AirQuality
    ```

5.  Monitor messages from the board.
    * Replace `<port>` below with the port for the teensy board.
    ```
    arduino-cli monitor --port <port>
    ```
