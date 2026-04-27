# BadgeBot app

Companion app for the HexDrive hexpansion. Supports 2 brushed DC motors, 4 RC servos, 1 motor + 2 servos. Features Logo-style motor programming, PID line following with automatic gain tuning, I²C sensor testing, servo test mode, and persistent settings management.

This guide is current for BadgeBot version 1.5

As this application has become quite complicated if you are looking for example code to use a HexDrive please see [HexDriveUseTemplate](https://github.com/TeamRobotmad/HexDriveUseTemplate)

## User guide

Install the BadgeBot app and then plug your HexDrive board into any of the hexpansion slots on your EMF Camp 2024 Badge.  If your HexDrive EEPROM has not been initialised before you will be promted to confirm that the hexpansion is a HexDrive, if you have other hexpansions plugged in which have uninitialised EEPROMs then please be careful to only initialise the correct one as being a HexDrive.

If your HexDrive software (stored on the EEPROM on the hexpansion) is not the latest version then you will be prompted to update this.  You can select from 5 'flavours' of configuration suitable for:
- 2 Motor
- 4 Servo
- 1 Motor and 2 Servos
- Unknown

The board can drive 2 brushed DC motors, 4 RC servos, 1 DC motor and 2 servos.
Once you have selected the desired 'flavour' - please confirm by pressing the "C" (confirm) button.
 
There must be a HexDrive board plugged in and running the latest software to use the BadgeBot app. If this is not the case then you will see a warning that you need a HexDrive with a reference to this repo. 

### Main Menu ###

The main menu presents the following options:
- **Line Follower** – PID-controlled line following using a HexSense with QTRX reflectance sensors
- **Motor Moves** – Logo/turtle-style motor programming (record UP/DOWN/LEFT/RIGHT sequences, then execute)
- **Servo Test** – Test up to 4 RC servos (position, trim, and scanning modes)
- **PID Auto Tune** – Automatic PID gain tuning using relay feedback (Åström-Hägglund method)
- **Settings** – Adjust configurable parameters (see below)
- **About** – Show version info, animated logo and QR code
- **Exit** – Exit the BadgeBot app

### Settings ###

The main menu includes a sub-menu of Settings which can be adjusted.
#### Motor Moves Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| acceleration     | Limits the change in motor drive per tick | 7500           | 1      | 65535  |
| max_power        | Maximum motor power level                 | 20000          | 1000   | 65535  |
| drive_step_ms    | Step duration for driving in ms           | 50             | 5      | 200    |
| turn_step_ms     | Step duration for turning in ms           | 20             | 5      | 200    |
#### Servo Test Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| servo_step       | Servo pulse step value in us              | 10             | 1      | 100    |
| servo_range      | Range of servo motion in us               | 1000           | 100    | 1400   |
| servo_period     | Servo period duration in ms               | 20             | 5      | 50     |
#### Line Follower Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| line_threshold   | Line sensor threshold                     | 500            | 0      | 65535  |
| pid_kp           | Proportional gain for line following      | 20000          | 0      | 65536  |
| pid_ki           | Integral gain for line following          | 0              | 0      | 65535  |
| pid_kd           | Derivative gain for line following        | 0              | 0      | 65535  |
#### Other Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| brightness       | LED brightness                            | 1.0            | 0.1    | 1.0    |
| logging          | Enable or disable logging                 | False          | False  | True   |

The PID gains are best set by using the "PID Auto Tune" menu option.  Place the robot on a line and press C to start the tuning process.  The auto-tuner uses relay feedback (Åström-Hägglund method) to determine the ultimate gain and period of oscillation, then calculates PID gains using Ziegler-Nichols tuning rules.  The tuning process includes a quality score (0-100%) indicating how consistent the oscillation data was.  Results are automatically saved to settings.

The training line should ideally include gentle curves so that the controller is exercised across a range of error magnitudes, but a straight line will also work for basic tuning.

### Limitations ###

When running from badge power the current available is limited - the best way to cope with this is to use low power motors and most importantly to limit the rate of change of the PWM signal, particularly avoiding rapid change of direction. The ```acceleration``` setting provides control of this in the BadgeBot application.

The maximum allowed servo range is VERY WIDE - most Servos will not be able to cope with this, so you probably want to reduce the ```servo_range``` setting to suit your servos.

Each Servo or Motor driver requires a PWM signals to control it, so a single HexDrive takes up four PWM resources on the ESP32.  As there are 8 such resources, the 'flavour' of your HexDrives will determine how many you can run simultaneously as long as you don't have any other hexpansions or applications using PWM resources. Two '4 Servo' or 'Unknown' flavour HexDrives will use up all the available PWM channels, whereas you can run up to 4 HexDrives in '2 Motor' flavour. (While each motor driver does actually require two PWM signals we have been able to reduce this to one by swapping it between the active signal when the motor direction changes.)

If you unplug a HexDrive the PWM resources will be released immediately so you can move them around the badge easily. 


### Install guide

Stable version available via [Tildagon App Directory](https://apps.badge.emfcamp.org/).

This repo contains lots of files that you don't need on your badge to use a HexDrive. If you want to load a minimal application onto a badge directly you only need the files (as long as you have already initialised the HexDrive EEPROM):
+ tildagon.toml
+ metadata.json
+ app.py or app.mpy
+ EEPROM/hexdrive.mpy
+ utils.mpy
+ hexpansion_mgr.mpy
+ motor_controller.mpy
+ motor_moves.mpy
+ servo_test.mpy
+ settings_mgr.mpy
+ line_follow.mpy
+ autotune.mpy
+ autotune_mgr.mpy
+ autodrive.mpy
+ sensor_manager.mpy
+ sensor_test.mpy
+ sensors/__init__.mpy
+ sensors/sensor_base.mpy
+ sensors/vl53l0x.mpy
+ sensors/vl6180x.mpy
+ sensors/tcs3472.mpy
+ sensors/tcs3430.mpy
+ sensors/opt4048.mpy


### Hexpansion Recovery ###

If you have issues with a HexDrive, or for that matter any hexpansion fitted with an EEPROM, e.g. a software incompatibility with a particular badge software version, you can reset the EEPROM back to blank as follows:
1) Plug in the hexpansion to Slot 1 (will work with any slot but you have to change the "1" below to the slot number.
2) Connect your favourite Terminal program to the COM port presented by the Badge over USB.
3) Press "Ctrl" & "C" simultaneously. i.e. "Ctrl-C" 
4) You should now be presented with a prompt ">>>" which is called the python REPL. At this type in the following lines (the HexDrive EEPROM is 8kbytes so requires 16 bit addressing, hence the ```addrsize=16``` other hexpansions may use smaller EEPROMS where this is not required):
   ```
		from machine import I2C
		i = I2C(1)
		i.writeto_mem(0x50, 0, bytes([0xFF]*8192), addrsize=16)
   ```
6) As long as there is no Traceback then this worked. But you can check by reading back the EEPROM contents with:
   ```
		i.readfrom_mem(0x50,0,32,addrsize=16)
   ```
	You should get a response which confirms that the first 32 bytes have been reset back to 0xFF:
```
    b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff'
```



### Construction guide & useful documents

https://github.com/TeamRobotmad/BadgeBotParts/tree/main/Docs


## Developers guide

Writing your own code to control the motor driver is very easy.  The BadgeBot application contains lots of extra code to support initialising and upgrading the software on the HexDrive, but once this is done you can use the board without needing this code.

To fit the HexDrive software into a small EEPROM it is converted into a .mpy file.  The file hexdrive.py is the source of this code if you want to see what it is doing.  The intention is that this code manages the hardware as it knows which slot the hexpansion is in.

### Power
The HexDrive incorporates a Switch Mode Power Supply which boosts the 3.3V provided by the badge up to 5V (or higher if your hexpansion has been modified) to drive the motors.  To turn this on or off call
```set_power(True | False)```

### Drive
Call ```set_motors()``` to control the two motors, providing a signed integer from -65535 to +65535 for each in a tuple.

Alternatively:
Call ```set_pwm()``` to set the duty cycle of the 4 PWM channels which control the motors. This function takes a tuple of 4 integers, each from 0 to 65535. e.g.
```set_pwm((0,1000,1000,0))```
note the extra set of brackets as the function argument is a single tuple of 4 values rather than being 4 individual values.

### Servos
You can control 1,2,3 or 4 RC hobby servos (centre pulse width 1500us).  The first time you set a pulse width for a channel using ```set_servoposition()``` the PWM frequency for that channel will be set to 50Hz.
The first two Channels take up signals that would otherwise control Motor 1 and the second two Channels take up the signals that are used for Motor 2.
You can use one motor and 1 or 2 servos simultaneously.

### Frequency
You can adjust the PWM frequency, default 20000Hz for motors and 50Hz for servos by calling the ```set_freq()``` function.

#### Keep Alive
To protect against most badge/software crashes causing the motors or servos to run out of control there is a keep alive mechanism which means that if you do not make a call to the ```set_pwm```, ```set_motors``` or ```set_servoposition``` functions the motors/servos will be turned off after 1000mS (default - which can be changed with a call to ```set_keep_alive()```).

### Developers setup
This is to help develop the BadgeBot application using the Badge simulator.

Windows:
```
git clone https://github.com/TeamRobotmad/BadgeBot.git
cd BadgeBot
powershell -ExecutionPolicy Bypass -File .\dev\setup_dev_env.ps1
```

WSL (recommended for simulator tests):
```
git clone https://github.com/TeamRobotmad/BadgeBot.git
cd BadgeBot
sh ./dev/setup_wsl_dev_env.sh
```

The WSL helper uses `uv` to provision Python 3.10 and installs both the local dev requirements and the simulator requirements. This is recommended because the published `wasmer` wheels used by the simulator currently load reliably there.

Linux/macOS:
```
git clone https://github.com/TeamRobotmad/BadgeBot.git
cd BadgeBot
sh ./dev/setup_dev_env.sh
```

If you prefer to run commands manually:
```
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r .\dev\dev_requirements.txt
```


### Running tests
Tests must be run from the `tests/` directory:
```
cd tests
python -m pytest test_smoke.py test_autotune.py -v
```

If BadgeBot is checked out inside the `badge-2024-software` repo, set `PYTHONPATH` to the parent repo root so `sim.run` can be imported. For the WSL helper's default environment this looks like:
```
cd tests
PYTHONPATH=/path/to/badge-2024-software ../.venv-wsl310/bin/python -m pytest test_smoke.py test_autotune.py -v
```

### Best practise
Run `isort` on in-app python files. Check `pylint` for linting errors.

### Regenerating QR Code
QR generation is a development-time task and is intentionally kept out of normal
runtime loading for the app.

Generate QR output only (prints `_QR_CODE = [...]`):
```
python dev/generate_qr_code.py --url https://robotmad.odoo.com
```

Generate and write directly back into `app.py`:
```
python dev/generate_qr_code.py --url https://robotmad.odoo.com --write-app
```

Optional: integrate into release prep:
```
python dev/build_release.py --refresh-qr --qr-url=https://robotmad.odoo.com
```

Validate `_QR_CODE` is in sync without modifying files:
```
python dev/check_qr_sync.py --url https://robotmad.odoo.com
```

`build_release.py` now checks QR sync by default before packaging.
Use `--no-check-qr` to skip this check if needed.


### Contribution guidelines
