""" GPS App for Hexpansion """
import app

from app_components.tokens import label_font_size, button_labels
from events.input import Buttons, BUTTON_TYPES, ButtonDownEvent
from system.eventbus import eventbus
from system.hexpansion.config import HexpansionConfig
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler.events import RequestForegroundPopEvent, RequestForegroundPushEvent, RequestStopAppEvent
from tildagonos import tildagonos
from machine import UART, Pin

# Minimal length method names to make the mpy file as small as possible so it might fit in the 2k hexpansion EEPROM.
# Minimal functionality to get a GPS fix
# This version is NOT for the App Store

VERSION = 1

# Hardware defintions:
TX_PIN  = 1    # HS_G for TX
RX_PIN  = 0    # HS_F for RX
RESET_PIN = 2  # HS_H for reset
PPS_PIN = 3    # HS_I for PPS

###JUST FOR USE WITH MY PROTOTYPE BOARD
ENABLE_PIN  = 0  # First LS pin used to enable the SMPSU
###JUST FOR USE WITH MY PROTOTYPE BOARD

class GPSApp(app.App):         # pylint: disable=no-member
    """ App to get GPS data from a GPS module connected to the hexpansion and display it on the badge. """
    def __init__(self, config: HexpansionConfig | None = None):
        super().__init__()
        # If run from EEPROM on the hexpansion, the config will be passed in with the correct pin objects
        self.config: HexpansionConfig | None = config
        if config is None:
            return
        self.tx_pin = config.pin[TX_PIN]
        self.rx_pin = config.pin[RX_PIN]
        self.reset = config.pin[RESET_PIN]
        self.pps = config.pin[PPS_PIN]

###JUST FOR USE WITH MY PROTOTYPE BOARD
        self.power_control = config.ls_pin[ENABLE_PIN]
        self.power_control.init(mode=Pin.OUT)
        self.power_control.value(1)
###JUST FOR USE WITH MY PROTOTYPE BOARD

        self.foreground = False
        self.button_states = Buttons(self)
        self.last_fix = None

        # Event handlers for gaining and losing focus and for stopping the app
        eventbus.on_async(RequestStopAppEvent, self.s, self)
        eventbus.on_async(RequestForegroundPushEvent, self.r, self)
        eventbus.on_async(RequestForegroundPopEvent, self.p, self)

        self.uart = UART(1, baudrate=9600, tx=self.tx_pin, rx=self.rx_pin)
        self.reset.init(mode=Pin.OUT)
        self.pps.init(mode=Pin.IN)
        self.reset.value(1) # set reset high here and release when 100ms has passed in foreground update.
        self.ticks_since_start = 0
        self.ticks_since_last_fix = 0


    def deinit(self):
        """ Deinitialise the app, releasing any resources (e.g. UART) """
        self.uart.deinit()
        self.power_control.value(0)  # Cut power to the GPS to save power when not in use


    async def s(self, event: RequestStopAppEvent):
        """ Handle the RequestStopAppEvent so that we can release resources """
        if event.app == self:
            self.deinit()


    async def r(self, event: RequestForegroundPushEvent):
        """ Handle the RequestForegroundPushEvent to know when we gain focus """
        if event.app == self:
            eventbus.emit(PatternDisable())
            eventbus.on(ButtonDownEvent, self.d, self)
            self.foreground = True


    async def p(self, event: RequestForegroundPopEvent):
        """ Handle the RequestForegroundPopEvent to know when we lose focus """
        if event.app == self:
            eventbus.emit(PatternEnable())
            eventbus.remove(ButtonDownEvent, self.d, self)


    def d(self, event: ButtonDownEvent):
        """ Handle button down events """
        if event.button == BUTTON_TYPES["CANCEL"]:
            self.button_states.clear()
            self.minimise()


    def update(self, delta):
        """ Update the app state - expire last_fix if it is too old """
        if self.reset.value():
            self.ticks_since_start += delta
            if self.ticks_since_start > 100:
                # Release reset after 100ms to allow the GPS to start up
                self.reset.value(0)
        if not self.foreground:
            # This triggers the automatic foreground display
            eventbus.emit(RequestForegroundPushEvent(self))
            self.foreground = True
        if self.last_fix:
            self.ticks_since_last_fix += delta
            if self.ticks_since_last_fix > 10000:
                # If it's been more than 10 seconds since the last fix, disccard it
                self.last_fix = None


    def background_update(self, _delta):
        """ Update in the background - read from the UART and parse any GPS data """
        line = self.uart.readline()
        if line:
            try:
                line = line.decode().strip()
                result = n(line)
                if result:
                    self.last_fix = result
                    self.ticks_since_last_fix = 0
            except (UnicodeError, ValueError, AttributeError):
                pass


    def draw(self, ctx):
        """ Draw the app - display the last GPS fix or a searching message if no fix is available """
        ctx.rgb(0, 0.2, 0).rectangle(-120, -120, 240, 240).fill()
        ctx.rgb(0, 1, 0)
        ctx.font_size = label_font_size
        ctx.text_align = ctx.LEFT
        ctx.text_baseline = ctx.BOTTOM
        if self.last_fix:
            ctx.move_to(-100, -10).text("Lat: " + str(round(self.last_fix["lat"], 5)))
            ctx.move_to(-100, 20).text("Lon: " + str(round(self.last_fix["lon"], 5)))
            for i in range(1, 13):
                tildagonos.leds[i] = (0,1,0)
            tildagonos.leds.write()
        else:
            ctx.move_to(-100, 0).text("Searching...")
            for i in range(1,13):
                tildagonos.leds[i] = (0,0,0)
            tildagonos.leds.write()

        # show labels for buttons
        button_labels(ctx, cancel_label="Exit")

def n(line: str) -> dict[str, float] | None:
    """ Parse an NMEA RMC sentence and return a dictionary with the latitude and longitude if valid, or None if invalid. """
    parts = line.split(',')

    if parts[0] not in ("$GNRMC", "$GPRMC"):
        return None
    elif parts[2] != "A":  # A = valid, V = invalid
        return None
    else:
        lat_raw = parts[3]
        lat_dir = parts[4]
        lon_raw = parts[5]
        lon_dir = parts[6]

        if not lat_raw or not lon_raw:
            return None

    # Convert to decimal degrees
        lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60
        lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60

        if lat_dir == "S":
            lat = -lat
        if lon_dir == "W":
            lon = -lon

        return {
            "lat": lat,
            "lon": lon
        }


__app_export__ = GPSApp     #pylint: disable=invalid-name
