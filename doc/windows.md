# Windows Setup

Open the Command Prompt app, found under Windows System in the Start menu.
* To copy: drag your mouse to select some text, then click with the 2nd mouse button.
* To paste: click with the 2nd mouse button.
* To repeat a previous command,
  use the up and down arrow keys to scroll through your command history.
  Use the backspace, delete, left arrow, and right arrow keys to edit the command.
  Use the enter key to execute the edited command.

![cmd](cmd.png)

1.  Delete (or rename) the following directories (if they exist)
    to remove any outdated components of the development environment.
    ```
    rmdir /s /q %LOCALAPPDATA%\Arduino15
    rmdir /s /q %USERPROFILE%\Documents\Arduino
    ```

    ![rmdir](rmdir.png)

2.  Download the 64-bit Windows MSI installler
    for the latest release of [arduino-cli](https://docs.arduino.cc/arduino-cli/installation/#download).
    (downloads less than 20 MB)
    * Launch the MSI installer.
    * After installation is compmlete, restart your computer.

3.  Download the [code](https://github.com/tmparks/arduino/archive/refs/heads/main.zip)
    for this project.
    (downloads less than 1 MB)
    * To extract all files from the zip archive:
      click with the 2nd mouse button and select Extract All... from the menu.
    * Move the resulting directory to a convenient location.

    ![extract](extract.png)
    ![move](move.png)

4.  Setup the development environment.
    (downloads less than 150 MB)
    * Click in the address bar of File Explorer and hit `CTRL-C`
      to select and copy the full path of
      the directory containing the files from the previous step.
    * Navigate to the directory (replacing `<dir>` below by pasting into Command Prompt)
      and run the setup script.
    ```
    chdir <dir>
    setup.bat
    ```

    ![setup](setup-1.png)
    ![setup](setup-2.png)

5.  Edit the file AirQuality.ino in the AirQuality directory
    to modify `BOX_NUMBER` and other configuration parameters.

    ![edit](edit.png)

6.  Compile the AirQuality sketch.
    (downloads less than 1 MB, first time only)
    ```
    arduino-cli compile AirQuality
    ```

    ![compile](compile.png)

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
