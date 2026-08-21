# CRP_QA_QC
Charge Readout Planes repo for QA QC code


# CTS Arduino Control System - README

Complete setup and operation guide for the LSU CTS (Cryogenic Test System) Arduino control code.

FAQ can be found here: https://docs.google.com/document/d/1Mvxc-8RMR9IKTzCEFsOGMOo0UzxVbzmNyf-6UGv6w-g/edit?usp=sharing

---

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Installation](#installation)
3. [Hardware Setup](#hardware-setup)
4. [Configuration](#configuration)
5. [Running the System](#running-the-system)
6. [Troubleshooting](#troubleshooting)
7. [Project Structure](#project-structure)

---

## System Requirements

### Hardware
- **Arduino Mega 2560 R3** microcontroller board
- **USB cables** to connect Arduino to PC
- **Linux PC** (Ubuntu 20.04 LTS or newer recommended)

### Software
- Linux operating system (Ubuntu/Debian-based)
- Python 3.7+
- Arduino CLI
- avrdude

---

## Installation

### Step 1: Install System Dependencies

```bash
# Update package manager
sudo apt update
sudo apt upgrade -y

# Install Python 3 and pip
sudo apt install -y python3 python3-pip

# Install Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install avrdude (for firmware uploads)
sudo apt install -y avrdude

# Install build tools (if needed)
sudo apt install -y gcc g++ make
```

### Step 2: Add User to dialout Group (Serial Port Access)


```
# Add current user to dialout group for serial port access
sudo usermod -a -G dialout $USER
```

# Apply group changes (choose one):
# Option A: Log out and log back in completely
# Option B: Use this command and reopen terminal
```bash
su - $USER
```

### Step 3: Clone the Repository

```bash
# Navigate to desired directory
cd ~/projects  # or any location you prefer

# Clone the repo
git clone https://github.com/your-username/CTS-Control-Code.git
cd CTS-Control-Code
```

### Step 4: Install Python Dependencies

```bash
# Install required Python packages
pip3 install pyserial

# Verify installation
python3 -c "import serial; print(f'pyserial version: {serial.__version__}')"
```

### Step 5: Configure Arduino CLI

```bash
# Initialize Arduino CLI configuration
arduino-cli config init

# Install the Arduino AVR boards package
arduino-cli core install arduino:avr

# Install required libraries
arduino-cli lib install "Adafruit AHTX0"
arduino-cli lib install "Adafruit ADS1X15"
arduino-cli lib install "TimerOne"
arduino-cli lib install "LiquidCrystal"

# Verify board is recognized
arduino-cli board list
```

You should see something like:

```python
Port     Protocol Type              Board Name FQBN            Core
/dev/ttyACM0 serial   Serial Port (USB) Arduino Mega 2560 arduino:avr:mega

```

Hardware Setup
USB Connection

    Connect Arduino to PC via USB-C cable (use direct connection, not through a hub if possible)
        A powered USB hub can be used, but direct connection is more reliable
        Two USB-C cables connected directly to the PC work best

Verify Connection

```bash
# Check if Arduino is detected
lsusb | grep Arduino

# Expected output:
# Bus 001 Device XXX: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)

# Find the serial port
ls -la /dev/ttyACM*

# Expected output:
# crw-rw---- 1 root dialout 166, 0 Aug 12 11:49 /dev/ttyACM0
```

### Pin Connections

⚠️ IMPORTANT: Ensure all external pin connections are properly attached before running the firmware. The system was tested with all pins connected.

If troubleshooting upload issues, temporarily disconnect all peripheral connections to the Arduino.
Configuration
Default Serial Port

The system automatically uses /dev/ttyACM0 as the serial port. If your Arduino is on a different port:

    Edit scripts/connect (line with SERIAL_PORT)
    Or check available ports with: ls /dev/ttyACM*

Baud Rate

The Arduino firmware uses 115200 baud rate. This is configured in:

    Arduino code: firmware/firmware.ino line 358: Serial.begin(115200);
    Upload script: scripts/connect (automatically configured)

Running the System
Start the CTS Control System

```bash
# Navigate to project directory
cd ~/path/to/CTS-Control-Code

# Make script executable (first time only)
chmod +x scripts/connect

# Start the system
./scripts/connect CTS
```

Expected Output

```markdown
=========================================================================
Checking Arduino Firmware...
=========================================================================
Current firmware checksum: b381bd7f48e42265047064bfc7a53685
Sketch uses 32788 bytes (12%) of program storage space.

Uploading to Arduino...
avrdude: writing flash (32788 bytes):
Writing | ################################################## | 100% 5.28s
avrdude: 32788 bytes of flash verified

✓ Firmware uploaded successfully!
=========================================================================

Starting Arduino Logger for CTS...
Connecting to /dev/ttyACM0...
Logging started.
--------------------------------------------------

Logger running in background (PID: 1705524).
Launching Real-Time Plotter...

Entering interactive mode. Type 'exit' or 'quit' to close everything.
------------------------------------------------------------------
Arduino Command > 
```

### Send Commands to Arduino

Once connected, type commands at the prompt:

```r
Arduino Command > T           # Example command
[CMD SENT] T

Arduino Command > exit        # Exit system

Output Files

Data is logged to: CTS/logs/CTS_Arduino_serial_log_YYYY-MM-DD-HHMM-HHMM.csv
```

### Stop the System

```shell
# In the interactive prompt, type:
Arduino Command > exit

# Or press Ctrl+C to force exit
^C
```

### Troubleshooting
Issue 1: "Permission denied" for /dev/ttyACM0

Error:

```javascript
Error opening serial port: [Errno 13] Permission denied: '/dev/ttyACM0'
```

Solution:

```bash
# Ensure you're in the dialout group
groups $USER

# Should show: dialout

# If not, add and then completely log out/log in:
sudo usermod -a -G dialout $USER
su - $USER  # Or restart terminal
```

Issue 2: Arduino Not Detected

Error:

```bash
ls: cannot access '/dev/ttyACM0': No such file or directory
```

### Solution:

```bash
# Check if Arduino is connected
lsusb | grep Arduino

# If not listed, try:
# 1. Unplug and replug USB cable
# 2. Try a different USB port
# 3. Try a different USB cable
# 4. Check device manager on Windows to verify hardware is working

# Check all available serial ports
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

Issue 3: Timeout During Firmware Upload

Error:

```bash
avrdude: stk500v2_ReceiveMessage(): timeout
```

Solutions (in order):

```bash
# 1. Disconnect all peripheral connections to Arduino pins
# (sensors, power connections, etc.)

# 2. Verify serial connection works
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0', 115200); print('Connected!')"

# 3. Check if another process is using the port
ps aux | grep serial_logger
ps aux | grep arduino

# Kill any processes if needed
pkill -f serial_logger

# 4. Try manual upload with verbose output
avrdude -C /etc/avrdude.conf -v -p atmega2560 -c wiring -P /dev/ttyACM0 -b 115200 -D -U flash:w:firmware.hex:i
```

Issue 4: "No such file or directory" for firmware files

Error:

```python
Error: Compiled .hex file not found
```
Solution:

```bash
# Verify project structure
ls -la firmware/
# Should show: firmware.ino, LSU_CTS_control_v1.ino, AD7746.h, AD7746_I2C.h

# Verify you're in correct directory
pwd
# Should end with: .../CTS-Control-Code/LSU_CTS_control_code

# Rebuild
rm -rf ~/.arduino15/cache/arduino/sketches/*
./scripts/connect CTS
```

Issue 5: "arduino-cli not found"

Error:

```bash
arduino-cli: command not found
```

Solution:

```bash
# Reinstall Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Add to PATH
export PATH=$PATH:~/bin

# Make permanent
echo 'export PATH=$PATH:~/bin' >> ~/.bashrc
source ~/.bashrc

# Verify
arduino-cli version
```

Issue 6: Python Module Import Errors

Error:

```vbnet
ModuleNotFoundError: No module named 'serial'
```
Solution:

```bash
# Install pyserial
pip3 install pyserial

# Or with sudo if needed
sudo pip3 install pyserial

# Verify
python3 -c "import serial; print(serial.__version__)"
```

### Project Structure

```perl
CTS-Control-Code/
├── LSU_CTS_control_code/          # Main project directory
│   ├── firmware/                  # Arduino firmware
│   │   ├── firmware.ino           # Main sketch (wrapper)
│   │   ├── LSU_CTS_control_v1.ino # Original control code
│   │   ├── AD7746.h               # Sensor header
│   │   └── AD7746_I2C.h           # I2C communication header
│   │
│   ├── src/                       # Python source code
│   │   ├── serial_logger.py       # Reads from Arduino, logs data
│   │   └── realtime_plotter.py    # Real-time data visualization
│   │
│   ├── scripts/                   # Bash scripts
│   │   └── connect                # Main entry point (run this!)
│   │
│   ├── logs/                      # Data output directory
│   │   └── CTS_Arduino_serial_log_*.csv  # Logged data files
│   │
│   ├── README.md                  # This file
│   └── .gitignore                 # Git ignore rules
│
└── .git/                          # Git repository metadata
```
### Features

✅ Automatic Firmware Detection

    Checks firmware integrity on every run
    Automatically uploads new firmware if changed
    Stores checksums to avoid unnecessary uploads

✅ Portable Paths

    Works on any Linux machine without path modifications
    Automatically finds all files relative to script location

✅ Real-Time Data Logging

    Serial data logged to timestamped CSV files
    Real-time plotting visualization
    Interactive command interface

✅ Safe Permissions

    No sudo required for normal operation
    Serial port access via dialout group
    Optional sudo-less avrdude uploads

Common Workflow

```bash
# 1. Clone repo (first time only)
git clone <repo-url>
cd CTS-Control-Code/LSU_CTS_control_code

# 2. Start system (every time)
./scripts/connect CTS

# 3. System starts automatically with:
#    - Firmware check & upload (if needed)
#    - Serial logger recording data
#    - Real-time plotter displaying data
#    - Interactive command prompt

# 4. Send commands as needed
Arduino Command > <your-command>

# 5. Exit when done
Arduino Command > exit
```

### Getting Help

If issues persist:

    Check the Troubleshooting section above
    Verify all installation steps were completed
    Check that Arduino is physically connected
    Review output of ./scripts/connect CTS for error messages
    Check log files in logs/ directory

System Requirements Summary
Component	Version	Installation
Ubuntu	20.04+	-
Python	3.7+	sudo apt install python3 python3-pip
Arduino CLI	Latest	curl ... | sh
avrdude	6.3+	sudo apt install avrdude
pyserial	3.5+	pip3 install pyserial
Notes

    ⚠️ First Run: System will compile and upload firmware. This may take 30-60 seconds.
    ⚠️ Serial Port: Ensure /dev/ttyACM0 exists and you have permissions
    💡 Pro Tip: Bookmark this README for quick reference
    📊 Data: Check logs/ directory for CSV output files

License & Attribution

This CTS Control Code repository is maintained by the LSU Physics Department and MIT Laboratory of Nuclear Science and Engineering.

For issues contact: Thomas Barbera (thomas03@mit.edu) or Cecilia Ferrari (ferraric@mit.edu)

Last Updated: August 12, 2026

Ready to go! 🚀 Run ./scripts/connect CTS and enjoy your CTS control system!
