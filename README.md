# BadgeBot app

Companion app for HexDrive expansion, assuming BadgeBot configuration with 2 motors.

## User guide

Install the BadgeBot app and then plug your HexDrive board into any of the hexpansion slots on your EMF Camp 2024 Badge.  If your HexDrive EEPROM has not been initialised before you will be promted to confirm that the hexpansion is a HexDrive, if you have other hexpansions plugged in which have uninitialised EEPROMs then please be careful to only initialise the correct one as being a HexDrive.

If your HexDrive software (stored on the EEPROM on the hexpansion) is not the latest version then you will be prompted to update this - please confirm by pressing the "C" (confirm) button.

There must be a HexDrive board plugged in and running the latest software to use the BadgeBot app. If this is not the case then you will see a warning that you need a HexDrive with a reference to this repo. 

Each motor driver requires two PWM signals to control it, so a single HexDrive takes up four PWM resources on the ESP32.  As there are 8 such resources you can run two HexDrives simultaneously as long as you don't have any other hexpansions or applications using PWM resources.

If you unplug a HexDrive the PWM resources will be released immediately so you can move them around the badge easily. 


### Install guide

Stable version available via [Tildagon App Directory](https://apps.badge.emfcamp.org/).

This repo contains lots of files that you don't need on your badge to use a HexDrive. If you want to load a minimal application onto a badge directly you only need the files (as long as you have already initialised the HExDrive EEPROM):
+ app.py
+ tildagon.toml
+ metadata.json
+ utils.py


### Construction guide & useful documents

https://github.com/TeamRobotmad/BadgeBotParts/tree/main/Docs


## Developers guide

Writing your own code to control the motor driver is very easy.  The BadgeBot application contains lots of extra code to support initialising and upgrading the software on the HexDrive, but once this is done you can use the board without needing this code.

To fit the HexDrive software into a small EEPROM it is converted into a .mpy file.  The file hexdrive.py is the source of this code if you want to see what it is doing.  The intention is that this code manages the hardware as it knows which slot the hexpansion is in.

### Power
The HexDrive incorporates a Switch Mode Power Supply which boosts the 3.3V provided by the badge up to 5V (or higher if your hexpansion has been modified).  To turn this on call
```set_power(True)```

### Drive
Call ```set_motors()``` to control the two motors, providing a signed integer from -65555 to +65535 for each in a list.

Alternatively:
Call ```set_pwm()``` to set the duty cycle of the 4 PWM channels which control the motors. This function takes a tuple of 4 integers, each from 0 to 65535. e.g.
```set_pwm((0,1000,1000,0))```
note the extra set of brackets as the function argument is a single tupple of 4 values rather than being 4 individual values.


To protect against most badge/software crashes causing the motors to run out of control there is a keep alive mechanism which means that if you do not make a call to the ```set_pwm``` fucntion the motors will be turned off after 1000mS (default - which can be changed with a call to ```set_keep_alive()```).

You can adjust the PWM frequency, default 20000Hz by calling the ```set_freq()``` function.

### Servos
You can control 1,2,3 or 4 RC hobby servos (centre pulse width 1500us).  The first time you set a pulse width for a channel using ```set_servo()``` the PWM frequency for that channel will be set to 50Hz.
Channels 0 & 1 take up signals that would otherwise control Motor 1 and Channels 2 & 3 take up the channels that are used for Motor 2.
You can use one motor and 1 or 2 servos simultaneously.

### Developers setup
```
git clone https://github.com/TeamRobotmad/badge-2024-software.git
cd badge-2024-software.git
git submodule update --init
pip install --upgrade pip
pip install -r ./sim/requirements.txt
pip install -r ./sim/apps/BadgeBot/dev/dev_requirements.txt
```


### Running tests
```
pytest test_smoke.py
```

### Best practise
Run `isort` on in-app python files. Check `pylint` for linting errors.


### Contribution guidelines
