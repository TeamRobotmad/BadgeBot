"""Shared pytest fixtures for BadgeBot tests.

Provides a generic fake-hexpansion framework that allows tests to simulate
the presence of any hexpansion type (HexDrive, HexSense, etc.) without
real hardware.  The mechanism patches the ``read_header()`` function so that
``HexpansionMgr._check_port_for_known_hexpansions`` finds a valid EEPROM
header, and injects a lightweight stub app into ``scheduler.apps`` so that
``_find_hexpansion_app`` succeeds.

Usage in tests::

    def test_something(badgebot_app_with_hexpansion):
        # 2-Motor HexDrive on port 1 by default
        app = badgebot_app_with_hexpansion
        assert app.num_motors == 2

    @pytest.mark.parametrize("hexdrive_pid", [0xCBCE])
    def test_stepper(badgebot_app_with_hexpansion):
        app = badgebot_app_with_hexpansion
        assert app.num_steppers == 1

The core helper :func:`install_fake_hexpansion` is deliberately generic –
callers supply a ``(vid, pid)`` pair and a port number, and it takes care
of the rest.  It is a context manager that guarantees cleanup (stopping
patches and removing the fake app from ``scheduler.apps``) even if the
test raises.  Specific HexDrive sub-types are configured through
``hexdrive_pid`` / ``hexdrive_port`` fixtures that tests can override via
``pytest.mark.parametrize`` or by defining local fixtures.

.. note::

   We do **not** import ``sim.run`` at module level.  ``sim/run.py``
   replaces ``sys.meta_path`` entirely, which would break pytest's
   ``faulthandler`` plugin during the early configuration phase.  All
   simulator-dependent imports are deferred to :func:`_ensure_sim_initialized`,
   which is called lazily from fixtures at test-execution time.
"""

import contextlib
import sys

from unittest.mock import patch

import pytest

# ---- path setup (harmless; just extends sys.path) --------------------------
sys.path.append("../../../")

# ---------------------------------------------------------------------------
#  Lazy simulator initialisation
# ---------------------------------------------------------------------------

_sim_initialized = False


def _ensure_sim_initialized():
    """Import ``sim.run`` exactly once to set up simulator shims.

    This must **not** be called at module level – only from inside fixtures
    or test functions – because ``sim/run.py`` replaces ``sys.meta_path``
    and would prevent pytest from finding ``faulthandler`` during its own
    ``pytest_configure`` phase.
    """
    global _sim_initialized
    if not _sim_initialized:
        import sim.run  # noqa: F401 – side effect: configures sys.path & fakes
        _sim_initialized = True


# ---------------------------------------------------------------------------
#  Generic fake-hexpansion helpers
# ---------------------------------------------------------------------------

class FakeHexpansionHeader:
    """Minimal stand-in for ``HexpansionHeader`` returned by ``read_header``.

    Only the attributes that ``HexpansionMgr._check_port_for_known_hexpansions``
    inspects are provided (``vid`` and ``pid``).
    """

    def __init__(self, vid: int, pid: int):
        self.vid = vid
        self.pid = pid


class _FakeHexDriveApp:
    """Lightweight stub that satisfies ``_find_hexpansion_app`` checks.

    ``type(instance).__name__`` must equal ``app_name`` from the matching
    ``HexpansionType`` entry (e.g. ``"HexDriveApp"``).  The stub also
    exposes the tiny surface that ``_update_state_check`` probes:

    * ``config.port`` – the port number
    * ``get_version()`` – returns the current HEXDRIVE_APP_VERSION
    * ``get_status()`` – returns True (PWM ready)
    * ``set_motors()`` – no-op
    """

    def __init__(self, port: int, version: int):
        _ensure_sim_initialized()
        from system.hexpansion.config import HexpansionConfig
        self.config = HexpansionConfig(port)
        self._version = version

    def get_version(self) -> int:
        return self._version

    def get_status(self) -> bool:
        return True

    # Motor control no-ops
    def set_motors(self, outputs):
        pass

    def set_power(self, state):
        return True

    def set_freq(self, freq, channel=None):
        return True

    def set_logging(self, state):
        pass

    def initialise(self):
        return True


# We need the class name to match "HexDriveApp" for _find_hexpansion_app
HexDriveApp = type("HexDriveApp", (_FakeHexDriveApp,), {})


@contextlib.contextmanager
def install_fake_hexpansion(vid: int, pid: int, port: int,
                            app_class=None, app_version: int | None = None):
    """Context manager that patches ``read_header`` and ``scheduler.apps``
    so that a fake hexpansion of the given type appears on *port*.

    Cleanup (stopping patches and removing the fake app from
    ``scheduler.apps``) is guaranteed even if the test raises.

    Parameters
    ----------
    vid, pid : int
        VID/PID pair written into the fake EEPROM header.
    port : int
        Badge slot number (1-6) where the fake hexpansion sits.
    app_class : type, optional
        The stub class whose ``__name__`` must match the ``app_name`` field
        in the corresponding ``HexpansionType``.  Defaults to
        ``HexDriveApp`` (name = ``"HexDriveApp"``).
    app_version : int, optional
        Value returned by ``get_version()``.  If *None* it is imported from
        ``hexdrive.VERSION`` at call time.

    Yields
    ------
    fake_app
        The fake app instance injected into ``scheduler.apps``.
    """
    _ensure_sim_initialized()

    if app_class is None:
        app_class = HexDriveApp
    if app_version is None:
        from sim.apps.BadgeBot.hexdrive import VERSION
        app_version = VERSION

    fake_app = app_class(port, app_version)

    # Build a ``read_header`` replacement that returns the fake header for
    # *port* and raises OSError for all other ports (= no EEPROM present).
    def _fake_read_header(port_arg, *args, **kwargs):
        if port_arg == port:
            return FakeHexpansionHeader(vid, pid)
        raise OSError("no EEPROM")

    patches = []

    # Patch read_header in the hexpansion_mgr module's namespace
    p1 = patch("sim.apps.BadgeBot.hexpansion_mgr.read_header", side_effect=_fake_read_header)
    p1.start()
    patches.append(p1)

    # Inject the fake app into scheduler.apps
    from system.scheduler import scheduler
    if not hasattr(scheduler, '_original_apps'):
        scheduler._original_apps = list(scheduler.apps)
    scheduler.apps.append(fake_app)

    try:
        yield fake_app
    finally:
        for p in patches:
            p.stop()
        if fake_app in scheduler.apps:
            scheduler.apps.remove(fake_app)
        if hasattr(scheduler, '_original_apps'):
            scheduler.apps[:] = scheduler._original_apps
            del scheduler._original_apps


# ---------------------------------------------------------------------------
#  Reusable pytest fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def port():
    """Default hexpansion port for legacy tests."""
    return 1


@pytest.fixture
def hexdrive_port():
    """Port for the fake HexDrive (override via parametrize)."""
    return 1


@pytest.fixture
def hexdrive_pid():
    """PID for the fake HexDrive sub-type (override via parametrize).

    Default is 0xCBCA = "2 Motor" HexDrive.
    """
    return 0xCBCA


@pytest.fixture
def badgebot_app():
    """A bare BadgeBotApp with no fake hexpansions.

    No hexpansion is detected, so ``initialise_settings()`` is never called
    by the hexpansion manager and only the minimal base settings exist.
    """
    _ensure_sim_initialized()
    from sim.apps.BadgeBot import BadgeBotApp
    return BadgeBotApp()


@pytest.fixture
def badgebot_app_with_hexpansion(hexdrive_pid, hexdrive_port):
    """A BadgeBotApp that has detected a fake HexDrive on *hexdrive_port*.

    The fixture:
    1. Patches ``read_header`` so the HexpansionMgr finds a valid header.
    2. Injects a stub ``HexDriveApp`` into ``scheduler.apps``.
    3. Drives the hexpansion-management state-machine through enough
       ``update()`` calls for the app to reach STATE_MENU with all
       hardware-dependent settings registered.

    Yields the app instance; tears down patches on exit.
    """
    _ensure_sim_initialized()

    vid = 0xCAFE  # standard VID for all hexpansion types
    with install_fake_hexpansion(vid, hexdrive_pid, hexdrive_port):
        from sim.apps.BadgeBot import BadgeBotApp
        from sim.apps.BadgeBot.app import STATE_MENU
        app = BadgeBotApp()

        # Drive the state machine: the hexpansion_mgr starts in _SUB_INIT,
        # and needs several update() cycles to go through scan → check →
        # hexdrive port check → version check → settings init → menu.
        for _ in range(20):
            app.update(100)
            if app.current_state == STATE_MENU:
                break

        if app.current_state != STATE_MENU:
            hexpansion_mgr = getattr(app, "hexpansion_mgr", None)
            sub_state = getattr(hexpansion_mgr, "sub_state", None)
            pytest.fail(
                "badgebot_app_with_hexpansion did not reach STATE_MENU after "
                f"20 update() calls; final state={app.current_state}, "
                f"hexpansion sub_state={sub_state}"
            )

        yield app
