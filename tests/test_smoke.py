import sys
import pytest

# Add badge software to pythonpath
sys.path.append("../../../") 

import sim.run

def test_import_badgebot_app_and_app_export():
    from sim.apps.BadgeBot import BadgeBotApp
    import sim.apps.BadgeBot.app as BadgeBot
    assert BadgeBot.__app_export__ == BadgeBotApp

def test_import_hexdrive_app_and_app_export():
    from sim.apps.BadgeBot.hexdrive import HexDriveApp
    import sim.apps.BadgeBot.hexdrive as HexDrive
    assert HexDrive.__app_export__ == HexDriveApp

def test_badgebot_app_init():
    from sim.apps.BadgeBot import BadgeBotApp
    BadgeBotApp()

@pytest.mark.xfail
def test_hexdrive_app_init():
    from sim.apps.BadgeBot.hexdrive import HexDriveApp
    HexDriveApp()
