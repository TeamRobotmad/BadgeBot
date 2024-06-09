import sys

import pytest

# Add badge software to pythonpath
sys.path.append("../../../") 

import sim.run
from system.hexpansion.config import HexpansionConfig

def test_import_badgebot_app_and_app_export():
    import sim.apps.BadgeBot.app as BadgeBot
    from sim.apps.BadgeBot import BadgeBotApp
    assert BadgeBot.__app_export__ == BadgeBotApp

def test_import_hexdrive_app_and_app_export():
    import sim.apps.BadgeBot.hexdrive as HexDrive
    from sim.apps.BadgeBot.hexdrive import HexDriveApp
    assert HexDrive.__app_export__ == HexDriveApp

def test_badgebot_app_init():
    from sim.apps.BadgeBot import BadgeBotApp
    BadgeBotApp()

def test_hexdrive_app_init(port):
    from sim.apps.BadgeBot.hexdrive import HexDriveApp
    config = HexpansionConfig(port)
    HexDriveApp(config)

@pytest.fixture
def port():
    return 1
