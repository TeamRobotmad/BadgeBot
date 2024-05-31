import asyncio
from typing import Optional

import app
from enum import Enum

from app_components.tokens import label_font_size
from events.input import Buttons, BUTTON_TYPES

VERTICAL_OFFSET = label_font_size
H_START = -78
V_START = -58

class BadgeBotAppState(Enum):
    MENU = 1
    RECEIVE_INSTR = 2
    COUNTDOWN = 3
    RUN = 4

class Instruction:

    def __init__(self, press_type: BUTTON_TYPES) -> None:
        self._press_type = press_type
        self._duration = 1

    @property
    def press_type(self) -> BUTTON_TYPES:
        return self._press_type

    def inc(self):
        self._duration += 1

    def __str__(self):
        return f"{self.press_type.name} {self._duration}"

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

        # Overall app state
        self.current_state = BadgeBotAppState.MENU

    def update(self, delta):
        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            # The button_states do not update while you are in the background.
            # Calling clear() ensures the next time you open the app, it stays open.
            # Without it the app would close again immediately.
            self.button_states.clear()
            self.minimise()

        if self.current_state == BadgeBotAppState.MENU:
            # Exit start menu
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = BadgeBotAppState.RECEIVE_INSTR
                self.button_states.clear()


        elif self.current_state == BadgeBotAppState.RECEIVE_INSTR:
            # Enable/disable scrolling
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                if self.last_press != BUTTON_TYPES["CONFIRM"]:
                    self.is_scroll = not self.is_scroll
                    self.last_press = BUTTON_TYPES["CONFIRM"]
                else:
                    self.long_press_delta += delta
                    if self.long_press_delta >= 1000:
                        self.finalize_instruction()
                        self.current_state = BadgeBotAppState.COUNTDOWN
                return None
            self.long_press_delta = 0

            if self.is_scroll:
                self.last_press = BUTTON_TYPES["CANCEL"]

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

        elif self.current_state == BadgeBotAppState.COUNTDOWN:
            self.run_countdown_ms += delta
            if self.run_countdown_ms >= self.run_countdown_target_ms:
                self.current_state = BadgeBotAppState.RUN


    def _handle_instruction_press(self, press_type: BUTTON_TYPES):
        if self.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        self.last_press = press_type

    def draw(self, ctx):
        ctx.save()
        ctx.font_size = label_font_size
        # Scroll mode indicator
        if self.is_scroll:
            ctx.rgb(0.1,0,0).rectangle(-120,-120,240,240).fill()
        else:
            ctx.rgb(0,0,0.1).rectangle(-120,-120,240,240).fill()

        if self.current_state == BadgeBotAppState.MENU:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("To Program:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET).text("Press C")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 10).text("When finished:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 10).text("Long press C")
        elif self.current_state == BadgeBotAppState.RECEIVE_INSTR:
            for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
                ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET * (self.scroll_offset + i_num)).text(str(instr))
        elif self.current_state == BadgeBotAppState.COUNTDOWN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running in:")
            countdown_val = (self.run_countdown_target_ms - self.run_countdown_ms) / 1000
            ctx.rgb(1,1,0).move_to(H_START, V_START+VERTICAL_OFFSET).text(countdown_val)
        elif self.current_state == BadgeBotAppState.RUN:
            ctx.rgb(1,0,0).move_to(H_START, V_START).text("Running")
        ctx.restore()


    def finalize_instruction(self):
        if self.current_instruction is not None:
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                self.scroll_offset -= 1
            self.current_instruction = None

__app_export__ = BadgeBotApp
