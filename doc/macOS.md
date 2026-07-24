# macOS setup

1.  Delete (or rename) the following directories (if they exist)
    to remove any outdated components of the development environment.
    ```
    rm -rf ~/Library/Arduino15
    rm -rf ~/Documents/Arduino
    ```

2.  Download the 64-bit macOS ARM archive
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
