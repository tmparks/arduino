# Windows setup

Open the Command Prompt app, found under Windows System in the Start menu.
* To copy: drag the mouse to select some text, then click the 2nd mouse button.
* To paste: click the 2nd mouse button.
* To repeat a previous command,
  use the UP and DOWN arrow keys to scroll through the command history.
  Use the BACKSPACE, DELETE, LEFT arrow, and RIGHT arrow keys to edit the command.
  Use the ENTER key to execute the edited command
  or the ESCAPE key to clear the edited command.

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
    * After installation is complete, restart the computer.

3.  Download the [code](https://github.com/tmparks/arduino/archive/refs/heads/main.zip)
    for this project.
    (downloads less than 1 MB)
    * To extract all files from the zip archive:
      click the 2nd mouse button and select Extract All... from the menu.
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
