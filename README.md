# arduino

1.  Download and install [arduino-cli](https://docs.arduino.cc/arduino-cli/installation/#download)
    (downloads less than 20 MB)
    * Windows: download and launch msi installer
    * macOS or Linux terminal
      ```
      sudo mv arduino-cli /usr/local/bin
      ```

2.  Download and extract [code](https://github.com/tmparks/arduino/archive/refs/heads/main.zip)
    (downloads less than 1 MB)

3.  Setup development environment
    (downloads less than 150 MB)
    * Windows PowerShell
      ```
      .\setup.ps1
      ```
    * macOS or Linux terminal
      ```
      ./setup.sh
      ```

4.  Compile
    (downloads less than 1 MB, first time only)
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
