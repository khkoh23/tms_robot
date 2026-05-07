# Bringup for Transcranial Magnetic Stimulation Robot system

## Prerequisite
1. Make sure setserial is installed:\
`sudo apt install setserial`

2. Create / Add the following lines in __/etc/udev/rules.d/99-usb-serial.rules__:\
For FTS-300S:\
`SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="fts", RUN+="/bin/setserial /dev/%k low_latency"`\
For UC4:\
`SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="uc4"`\
