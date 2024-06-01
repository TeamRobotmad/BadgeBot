import app

from app_components.tokens import label_font_size
from app_components.notification import Notification
from events.input import Buttons, BUTTON_TYPES
from tildagonos import tildagonos

# Motor Driver
PWM_FREQ = 5000
MAX_DUTY = 1023

VERTICAL_OFFSET = label_font_size
H_START = -78
V_START = -58

TICK_MS = 20
POWER_STEP_PER_TICK = 20
LONG_PRESS_MS = 750

USER_TICK_MULTIPLIER = 4

MAX_POWER = 100

# App states
WARNING = 0
MENU = 1
RECEIVE_INSTR = 2
COUNTDOWN = 3
RUN = 4
DONE = 5

MINIMISE_VALID_STATES = [0, 1, 2, 5]

class Instruction:

    def __init__(self, press_type: BUTTON_TYPES) -> None:
        self._press_type = press_type
        self._duration = 1

        self.power_plan_iterator = iter([])

    @property
    def press_type(self) -> BUTTON_TYPES:
        return self._press_type

    def inc(self):
        self._duration += 1

    def __str__(self):
        return f"{self.press_type.name} {self._duration}"

    def directional_power_tuple(self, power):
        if self._press_type == BUTTON_TYPES["UP"]:
            return (power, 0, power, 0)
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (0, power, 0, power)
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (power, 0, 0, power)
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (0, power, power, 0)

    def make_power_plan(self):
        # return collection of tuples of power and their duration
        ramp_up = [(self.directional_power_tuple(p), TICK_MS)
                   for p in range(0, MAX_POWER, POWER_STEP_PER_TICK)]
        power_durations = ramp_up.copy()
        user_power_duration = TICK_MS * USER_TICK_MULTIPLIER * (self._duration-1)
        power_durations.append((self.directional_power_tuple(MAX_POWER), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        self.power_plan_iterator = iter(power_durations)


class BadgeBotApp(app.App):
    def __init__(self):
        self.button_states = Buttons(self)
        self.last_press: BUTTON_TYPES = BUTTON_TYPES["CANCEL"]
        self.long_press_delta = 0

        self.is_scroll = False
        self.scroll_offset = 0

        self.run_countdown_target_ms = 3000
        self.run_countdown_ms = 0

        self.instructions = []
        self.current_instruction = None

        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])

        self.notification = None

        # Overall app state
        self.current_state = WARNING

    def update(self, delta):
        if self.notification:
            self.notification.update(delta)

        if self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            self.button_states.clear()
            self.minimise()

        elif self.current_state == MENU:
            # Exit start menu
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = RECEIVE_INSTR
                self.button_states.clear()

        elif self.current_state == WARNING:
            # Exit warning screen
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = MENU
                self.button_states.clear()

        elif self.current_state == RECEIVE_INSTR:
            self.clear_leds()
            # Enable/disable scrolling and check for long press
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):

                if self.long_press_delta == 0:
                    # TODO Move to button up event
                    self.is_scroll = not self.is_scroll
                    self.notification = Notification(f"Scroll {self.is_scroll}")

                self.long_press_delta += delta
                if self.long_press_delta >= LONG_PRESS_MS:
                    self.finalize_instruction()
                    self.current_state = COUNTDOWN

            else:
                # Confirm is not pressed. Reset long_press state
                self.long_press_delta = 0

                # Manage scrolling
                if self.is_scroll:
                    if self.button_states.get(BUTTON_TYPES["DOWN"]):
                        self.scroll_offset -= 1
                    elif self.button_states.get(BUTTON_TYPES["UP"]):
                        self.scroll_offset += 1
                    self.button_states.clear()

                # Instruction button presses
                elif self.button_states.get(BUTTON_TYPES["RIGHT"]):
                    self._handle_instruction_press(BUTTON_TYPES["RIGHT"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["LEFT"]):
                    self._handle_instruction_press(BUTTON_TYPES["LEFT"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["UP"]):
                    self._handle_instruction_press(BUTTON_TYPES["UP"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["DOWN"]):
                    self._handle_instruction_press(BUTTON_TYPES["DOWN"])
                    self.button_states.clear()

                # LED management
                if self.last_press == BUTTON_TYPES["RIGHT"]:
                    tildagonos.leds[2] = (255, 0, 0)
                    tildagonos.leds[3] = (255, 0, 0)
                elif self.last_press == BUTTON_TYPES["LEFT"]:
                    tildagonos.leds[8] = (0, 255, 0)
                    tildagonos.leds[9] = (0, 255, 0)
                elif self.last_press == BUTTON_TYPES["UP"]:
                    tildagonos.leds[12] = (0, 0, 255)
                    tildagonos.leds[1] = (0, 0, 255)
                elif self.last_press == BUTTON_TYPES["DOWN"]:
                    tildagonos.leds[6] = (255, 255, 0)
                    tildagonos.leds[7] = (255, 255, 0)

            tildagonos.leds.write()

        elif self.current_state == COUNTDOWN:
            self.run_countdown_ms += delta
            if self.run_countdown_ms >= self.run_countdown_target_ms:
                self.power_plan_iter = chain(*(instr.power_plan_iterator for instr in self.instructions))
                self.current_state = RUN

        elif self.current_state == RUN:
            print(delta)
            power = self.get_current_power_level(delta)
            if power is None:
                self.current_state = DONE
            print(f"Using power: {power}")

        elif self.current_state == DONE:
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.reset()

    def _handle_instruction_press(self, press_type: BUTTON_TYPES):
        if self.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        self.last_press = press_type

    def reset(self):
        self.current_state = MENU
        self.button_states.clear()
        self.last_press: BUTTON_TYPES = BUTTON_TYPES["CANCEL"]
        self.long_press_delta = 0

        self.is_scroll = False
        self.scroll_offset = 0

        self.run_countdown_target_ms = 3000
        self.run_countdown_ms = 0

        self.instructions = []
        self.current_instruction = None

        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])


    def draw(self, ctx):
        ctx.save()
        ctx.font_size = label_font_size
        # Scroll mode indicator
        if self.is_scroll:
            ctx.rgb(0.1,0,0).rectangle(-120,-120,240,240).fill()
        else:
            ctx.rgb(0,0,0.1).rectangle(-120,-120,240,240).fill()

        if self.current_state == WARNING:
            ctx.rgb(1,1,1).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text("Please buy")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text("HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text("Hexpansion")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 20).text("from")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 4*VERTICAL_OFFSET + 20).text("RobotMad")
        elif self.current_state == MENU:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("To Program:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET).text("Press C")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 10).text("When finished:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 10).text("Long press C")
        elif self.current_state == RECEIVE_INSTR:
            for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
                ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET * (self.scroll_offset + i_num)).text(str(instr))
        elif self.current_state == COUNTDOWN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running in:")
            countdown_val = (self.run_countdown_target_ms - self.run_countdown_ms) / 1000
            ctx.rgb(1,1,0).move_to(H_START, V_START+VERTICAL_OFFSET).text(str(countdown_val))
        elif self.current_state == RUN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running power")
            ctx.rgb(1,0,0).move_to(H_START-30, V_START + 2*VERTICAL_OFFSET).text(str(self.current_power_duration))
        elif self.current_state == DONE:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text(f"Complete!")
            ctx.rgb(1,1,1).move_to(H_START, V_START + VERTICAL_OFFSET).text("To restart:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 2*VERTICAL_OFFSET).text("Press C")
        if self.notification:
            self.notification.draw(ctx)
        ctx.restore()


    def get_current_power_level(self, delta) -> int:
        # takes in delta as ms since last call
        # if delta was > 20... what to do
        if delta >= TICK_MS:
            delta = TICK_MS-1

        current_power, current_duration = self.current_power_duration

        updated_duration = current_duration - delta
        if updated_duration <= 0:
            try:
                next_power, next_duration = next(self.power_plan_iter)
            except StopIteration:
                # returns None when complete
                return None
            next_duration += updated_duration
            self.current_power_duration = next_power, next_duration
            return next_power
        else:
            self.current_power_duration = current_power, updated_duration
            return current_power


    def finalize_instruction(self):
        if self.current_instruction is not None:
            self.current_instruction.make_power_plan()
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                self.scroll_offset -= 1
            self.current_instruction = None

    def clear_leds(self):
        for i in range(0,12):
            tildagonos.leds[i+1] = (0, 0, 0)

def chain(*iterables):
    for iterable in iterables:
        yield from iterable

__app_export__ = BadgeBotApp
