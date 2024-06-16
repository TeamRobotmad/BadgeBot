# BadgeBot app

Companion app for HexDrive expansion, assuming BadgeBot configuration with 2 motors.

## User guide

### Install guide

Stable version available via [Tildagon App Directory](https://apps.badge.emfcamp.org/).

### Usage guide
https://drive.google.com/drive/folders/1DVCi_XbHi8Y9YzOkOgK-ow21TaVT0tyb?usp=sharing

## Developers guide

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
