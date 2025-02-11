# Setting Up Multiple Servo Controllers 

## Overview
If you are using multiple servo controller boards, you need to assign them unique device names to prevent conflicts. By default, they may appear as `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc., and their order may change after a reboot. This guide explains how to assign persistent names using **udev rules**.

## Method 1: Using udev Rules (Recommended)
### Step 1: Identify the Servo Controllers
Plug in both servo controllers **one at a time** and run the following command:
```bash
ls -l /dev/serial/by-id/
```
This will output something like:
```
usb-1a86_USB2.0-Serial-if00-port0 -> ../../ttyUSB0
usb-1a86_USB2.0-Serial-if00-port1 -> ../../ttyUSB1
```
Each device has a unique identifier. Note these names.

### Step 2: Create a udev Rule
Create a new udev rules file:
```bash
sudo nano /etc/udev/rules.d/99-servocontroller.rules
```
Add the following lines, replacing the `idVendor`, `idProduct`, and `serial` values with those from your system:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{serial}=="A1234567", SYMLINK+="servocontroller_1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{serial}=="B7654321", SYMLINK+="servocontroller_2"
```

### Step 3: Reload udev Rules
Run the following commands to apply the changes:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 4: Use the New Device Names
Now you can refer to your servo controllers using:
```
/dev/servocontroller_1
/dev/servocontroller_2
```
Update your Python script accordingly:
```python
DEVICENAME_1 = "/dev/servocontroller_1"
DEVICENAME_2 = "/dev/servocontroller_2"
```

## Method 2: Using `/dev/serial/by-id/` (Alternative)
If you prefer a simpler approach, you can use the device names directly from `/dev/serial/by-id/`:
```python
DEVICENAME_1 = "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
DEVICENAME_2 = "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port1"
```
However, this may cause issues if the order changes.

## Conclusion
Using **udev rules** ensures that each servo controller always has a fixed name, avoiding potential conflicts. If you need a quick solution, using `/dev/serial/by-id/` is an alternative, but not as reliable.

If you encounter any issues, double-check the `ls -l /dev/serial/by-id/` output and adjust the udev rules accordingly.

