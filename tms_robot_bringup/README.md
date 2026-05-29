# Bringup for Transcranial Magnetic Stimulation Robot system

## Prerequisite
1. Make sure setserial is installed:\
`sudo apt install setserial`

2. Create / Add the following lines in __/etc/udev/rules.d/99-usb-serial.rules__:\
For FTS-300S:\
`SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="fts", RUN+="/bin/setserial /dev/%k low_latency"`\
For UC4:\
`SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="uc4"`\

3. Setting up desktop shorcut icon:\
Ensure source script permission before colcon build:\
`chmod +x ~/tms_ws/src/tms_robot/tms_robot_bringup/desktop/run_all_launch.sh`\
Copy the shorcut template to the desktop:\
`cp ~/tms_ws/install/tms_robot_bringup/share/tms_robot_bringup/desktop/tms_robot.desktop.template ~/Desktop/tms_robot.desktop`\
Update the icon path to point to the install directory and make it executable:\
`sed -i "s|PLACEHOLDER_PATH|$HOME/tms_ws/install/tms_robot_bringup|g" ~/Desktop/tms_robot.desktop`\
Grant permission to the desktop shorcut
`chmod +x ~/Desktop/tms_robot.desktop`\
Right-click the desktop icon and select "Allow Launching"