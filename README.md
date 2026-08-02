# BadgeBot app

Companion app for the HexDrive hexpansion. Supports 2 brushed DC motors, 4 RC servos (2 for HexDrive2), 1 motor + 2 servos (1 for HexDrive2). Features Logo-style motor programming, line following with colour sensor, Bluetooth LE control, I²C sensor testing, servo test mode, and persistent settings management.

This guide is current for BadgeBot version 2.8

As this application has become quite complicated if you are looking for example code to use a HexDrive please see [HexDriveUseTemplate](https://github.com/TeamRobotmad/HexDriveUseTemplate)

See also the [HexDrive2 README](https://github.com/TeamRobotmad/HexDrive2)

## User guide

Install the BadgeBot app and then plug your HexDrive board into any of the hexpansion slots on your EMF Camp 2024/2026 Badge.  If your HexDrive EEPROM has not been initialised before you will be promted to confirm that the hexpansion is a HexDrive, if you have other hexpansions plugged in which have uninitialised EEPROMs then please be careful to only initialise the correct one as being a HexDrive.

If your HexDrive software (stored on the EEPROM on the hexpansion) is not the latest version then you will be prompted to update this.  You can select from 5 'flavours' of configuration suitable for:
- 2 Motor
- 2/4 Servo
- 1 Motor and 2 Servos
- Uncommited (This is how new boards are supplied)

Once you have selected the desired 'flavour' - please confirm by pressing the "C" (confirm) button.

There must be a HexDrive board plugged in and running the latest software to use the BadgeBot app. If this is not the case then you will see a warning that you need a HexDrive with a reference to this repo.

### Main Menu ###

The main menu presents the following options:
- **Bluetooth** - Bluetooth LE control (e.g. via Adafruit Bluefruit LE Connect Phone App)
- **Line Follower** – PID-controlled line following using colour sensor on HexDrive2
- **Motor Moves** – Logo/turtle-style motor programming (record UP/DOWN/LEFT/RIGHT sequences, then execute)
- **Sensor Test** - Test the Range and Colour sensor on the HexDrive2 - also used to calibrate the Colour sensor.
- **Servo Test** – Test up to 4 RC servos (position, trim, and scanning modes)
- **Settings** – Adjust configurable parameters (see below)
- **About** – Show version info, animated logo and QR code
- **Exit** – Exit the BadgeBot app

Note 1 - using the "CANCEL" button from the main menu does NOT exit the application, it is just "minimised" but remains running in memory - if you have finished using the App then it is best to properly Exit.
Note 2 - please use the Sensor Test feature to calibrate ("Cal") the Colour Sensor by placing over a black and then a white surface when prompted, BEFORE using the Line Follower.

### Settings ###

The main menu includes a sub-menu of Settings which can be adjusted.
#### Motors Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| acceleration     | Limits the change in motor power per tick | 7500           | 2      | 127    |
| deadband         | Motor power deadband                      | 1              | 0      | 127    |
| max_power        | Maximum motor power level                 | 107            | 20     | 127    |
| mtr1_dir         | Motor 1 direction                         | 0              | 0      | 1      |
| mtr2_dir         | Motor 2 direction                         | 0              | 0      | 1      |
| mtr1_min         | Motor 1 minimum power                     | 0              | 0      | 127    |
| mtr2_min         | Motor 2 minimum power                     | 0              | 0      | 127    |
#### Driving Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| front_face       | Badge edge treated as forward             | 0              | 0      | 11     |
| drive_step_ms    | Motor Moves step duration forward/backward| 50             | 10     | 10000  |
| turn_step_ms     | Motor Moves Step duration for turning     | 20             | 10     | 10000  |
| drive_mode       | Motor Moves drive mode                    | Time           | Time   |Distance|
#### Servos Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| servo_step       | Position adjustment step value in us      | 10             | 1      | 100    |
| servo_range      | Range of servo motion in us               | 1000           | 100    | 1400   |
| pwm_period       | Servo period duration in ms               | 20             | 5      | 50     |
#### Line Follower Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| mid_hue          | Colour Hue recognised as middle of line   | 300            | 0      | 360    |
| hue_range        | Colour Hue range recognised as 'line'     | 70             | 0      | 180    |
| line_power       | Motor power when following line           | 60             | 2      | 127    |
| pid_kp           | Proportional gain for line following      | 60             | 0      | 65536  |
| pid_ki           | Integral gain for line following          | 0              | 0      | 65535  |
| pid_kd           | Derivative gain for line following        | 25             | 0      | 65535  |
| stop_range       | Minimum distance to obstacle in mm        | 100            | 20     | 500    |
| stop_colour      | Colour that stops line following          | Black          |        |        |
| plot_type        | Data to show on live BLE Plot             | None           | None   | Power  |
|                  |   (None, Colour, Range, PID, Power)       |                |        |        |
#### General Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| brightness       | LED brightness                            | 1.0            | 0.1    | 1.0    |
| logging          | Enable or disable logging                 | False          | False  | True   |

If you are using the BadgeBot application to control a HexDrive then you will need to set the parameters in the Motors Settings section to suit your motors and servos.  The default values are suitable for low power motors and servos, but if you have more powerful motors then you will need to reduce the ```acceleration``` setting and possibly also the ```max_power``` setting.

If your Badgebot is not driving straight then you may need to adjust the ```mtr1_dir``` and ```mtr2_dir``` settings to reverse the direction of one or both motors.  If your motors are not starting to move until a high PWM value is reached then you will need to increase the ```mtr1_min``` and/or ```mtr2_min``` settings.  There is considerable resistance to small gear motors so the minimum PWM value to get them to move is often quite high.

### Limitations ###

When running from badge power the current available is limited - the best way to cope with this is to use low power motors and most importantly to limit the rate of change of the PWM signal, particularly avoiding rapid change of direction. The ```acceleration``` setting provides control of this in the BadgeBot application.

The maximum allowed servo range is VERY WIDE - most Servos will not be able to cope with this, so you probably want to reduce the ```servo_range``` setting to suit your servos.

Each Servo or Motor driver requires a PWM signal to control it, so a single HexDrive can take upto four PWM resources on the ESP32.  As there are 8 such resources, the 'flavour' of your HexDrives will determine how many you can run simultaneously as long as you don't have any other hexpansions or applications using PWM resources. Two '4 Servo' flavour HexDrives will use up all the available PWM channels, whereas you can run up to 4 HexDrives in '2 Motor' flavour. (While each motor driver does actually require two PWM signals we have been able to reduce this to one by swapping it between the active signal when the motor direction changes.)

If you unplug a HexDrive the PWM resources will be released immediately so you can move them around the badge easily.


### Install guide

Stable version available via [Tildagon App Directory](https://apps.badge.emfcamp.org/).

This repo contains lots of files that you don't need on your badge to use a HexDrive. If you want to load a minimal application onto a badge directly you only need the files (as long as you have already initialised the HexDrive EEPROM):
+ tildagon.toml
+ metadata.json
+ app.py or app.mpy
+ EEPROM/hexdrive.mpy
+ EEPROM/hexdrive2.mpy
+ utils.mpy
+ diagnostics.mpy
+ hexpansion_mgr.mpy
+ bluetooth_mgr.mpy
+ motor_controller.mpy
+ motor_moves.mpy
+ servo_test.mpy
+ settings_mgr.mpy
+ line_follow.mpy
+ sensor_test.mpy



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

### Minification

Hexpansion apps stored on EEPROM are minified before being compiled to `.mpy` to reduce their on-badge footprint.  The following files are minified:

| Source | Artifact |
|--------|----------|
| `vendor/HexDrive2/hexdrive2.py` | `EEPROM/hexdrive2.mpy` |
| `vendor/HexDrive/hexdrive.py` | `EEPROM/hexdrive.mpy` |

The pipeline uses `dev/minify.py` which:
1. Renames internal `self.*` attributes to short names via an AST transform (source stays readable)
2. Strips docstrings with `python-minifier`
3. Compiles with `mpy-cross -march=xtensawin -O3`

Typical savings are ~5% compared with compiling from source directly.

The minifier is invoked **automatically** by `dev/download_to_device.py` for any `ModuleSpec` that has `minify=True`.  You do not need to run it manually during normal development.

To run it standalone and see a before/after size comparison for all minified modules:
```
python dev/minify.py
```

Or to minify a single file (as `download_to_device.py` does):
```
python dev/minify.py --source vendor/HexDrive/hexdrive.py --artifact EEPROM/hexdrive.mpy
```

`python-minifier` is listed in `dev/dev_requirements.txt` and is installed as part of the standard dev-environment setup.

Intermediate build artefacts (`*.min.py`, `*.renamed.py`) are listed in `.gitignore` and should not be committed.

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
