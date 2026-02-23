# BadgeBot app

Companion app for HexDrive expansion, assuming BadgeBot configuration with 2 motors or 4 servos.

This guide is current for Badgebot version 1.3

As this application has become quite complicated if you are looking for example code to use a HexDrive please see [HexDriveUseTemplate](https://github.com/TeamRobotmad/HexDriveUseTemplate)

## User guide

Install the BadgeBot app and then plug your HexDrive board into any of the hexpansion slots on your EMF Camp 2024 Badge.  If your HexDrive EEPROM has not been initialised before you will be promted to confirm that the hexpansion is a HexDrive, if you have other hexpansions plugged in which have uninitialised EEPROMs then please be careful to only initialise the correct one as being a HexDrive.

If your HexDrive software (stored on the EEPROM on the hexpansion) is not the latest version then you will be prompted to update this.  You can select from 5 'flavours' of configuration suitable for:
- 2 Motor
- 4 Servo
- 1 Motor and 2 Servos
- Stepper
- Unknown

The board can drive 2 brushed DC motors, 4 RC servos, 1 DC motor and 2 servos or a single two phase Stepper Motor.
Once you have selected the desired 'flavour' - please confirm by pressing the "C" (confirm) button.
 
There must be a HexDrive board plugged in and running the latest software to use the BadgeBot app. If this is not the case then you will see a warning that you need a HexDrive with a reference to this repo. 

### Main Menu ###

The main menu will present options for a demonstration for a 2 motor robot "Motor Moves", a test/demo a single Stepper Motor "Stepper Test", or a simple "Servo Test" function for up to 4 servos.

### Settings ###

The main menu includes a sub-menu of Settings which can be adjusted.
#### Motor Moves SETTINGS ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| acceleration     | Limits the change in motor drive per tick | 7500           | 1      | 65535  |
| max_power        | Maximum Motor power level                 | 65536          | 1000   | 65535  |
| drive_step_ms    | Step duration for driving in ms           | 50             | 5      | 200    |
| turn_step_ms     | Step duration for turning in ms           | 20             | 5      | 200    |
#### Servo Test Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| servo_step       | Servo pulse step value in us              | 10             | 1      | 100    |
| servo_range      | Range of servo motion in us               | 1000           | 100    | 1400   |
| servo_period     | Servo period duration in ms               | 20             | 5      | 50     |
#### Other Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| brightness       | LED brightness                            | 1.0            | 0.1    | 1.0    |
| logging          | Enable or disable logging                 | False          | False  | True   |
| erase_slot       | Slot to offer erase function              | 0 (i.e. none)  | 0      | 6      |
| stepper_max_pos  | Maximum stepper position                  | 6200           | 0      | 65535  | 

### Limitations ###

When running from badge power the current available is limited - the best way to cope with this is to use low power motors and most importantly to limit the rate of change of the PWM signal, particularly avoiding rapid change of direction. The ```acceleration``` setting provides control of this in the BadgeBot application.

The maximum allowed servo range is VERY WIDE - most Servos will not be able to cope with this, so you probably want to reduce the ```servo_range``` setting to suit your servos.

Each Servo or Motor driver requires a PWM signals to control it, so a single HexDrive takes up four PWM resources on the ESP32.  As there are 8 such resources, the 'flavour' of your HexDrives will determine how many you can run simultaneously as long as you don't have any other hexpansions or applications using PWM resources. Two '4 Servo', 'Stepper' or 'Unknown' flavour HexDrives will use up all the available PWM channels, whereas you can run up to 4 HexDrives in '2 Motor' flavour. (While each motor driver does actually require two PWM signals we have been able to reduce this to one by swapping it between the active signal when the motor direction changes.)

If you unplug a HexDrive the PWM resources will be released immediately so you can move them around the badge easily. 


### Install guide

Stable version available via [Tildagon App Directory](https://apps.badge.emfcamp.org/).

This repo contains lots of files that you don't need on your badge to use a HexDrive. If you want to load a minimal application onto a badge directly you only need the files (as long as you have already initialised the HexDrive EEPROM):
+ app.py
+ tildagon.toml
+ metadata.json
+ utils.py


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
The HexDrive incorporates a Switch Mode Power Supply which boosts the 3.3V provided by the badge up to 5V (or higher if your hexpansion has been modified).  To turn this on call
```set_power(True)```

### Drive
Call ```set_motors()``` to control the two motors, providing a signed integer from -65555 to +65535 for each in a list.

Alternatively:
Call ```set_pwm()``` to set the duty cycle of the 4 PWM channels which control the motors. This function takes a tuple of 4 integers, each from 0 to 65535. e.g.
```set_pwm((0,1000,1000,0))```
note the extra set of brackets as the function argument is a single tupple of 4 values rather than being 4 individual values.


To protect against most badge/software crashes causing the motors or servos to run out of control there is a keep alive mechanism which means that if you do not make a call to the ```set_pwm```, ```set_motors```, ```motor_step``` or ```set_servoposition``` functions the motors/servos will be turned off after 1000mS (default - which can be changed with a call to ```set_keep_alive()```).

You can adjust the PWM frequency, default 20000Hz for motors and 50Hz for servos by calling the ```set_freq()``` function.

### Servos
You can control 1,2,3 or 4 RC hobby servos (centre pulse width 1500us).  The first time you set a pulse width for a channel using ```set_servo()``` the PWM frequency for that channel will be set to 50Hz.
The first two Channels take up signals that would otherwise control Motor 1 and the second two Channels take up the signals that are used for Motor 2.
You can use one motor and 1 or 2 servos simultaneously.

### Stepper Motor
You can control a single 2 phase stepper motor using ```motor_step()``` specifying which of the 8 possible phases to output in the range 0 to 7.  There are 8 possible values as half stepping is supported. To use only full steps specify phase values of 0, 2, 4 and 6.  Information on the pros and cons of using full or half stepping can be found online and what is right for you will depend on your motor and the application.  The motor can be released (so that it is not taking power to hold it in a fixed position) using ```motor_release()```.

### Developers setup
This is to help develop the BadgeBot application
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
