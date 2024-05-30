import asyncio
from tkinter import HORIZONTAL, Menu
from typing import Optional
import app
from enum import Enum

from app_components.tokens import label_font_size
from events.input import Buttons, BUTTON_TYPES

VERTICAL_OFFSET = label_font_size
HORIZONTAL_START = -80

class BadgeBotAppState(Enum):
    MENU = 1
    RECEIVE_INSTR = 2

class Instruction:

    def __init__(self, press_type: BUTTON_TYPES) -> None:
        self._press_type = press_type
        self._duration = 1

    @property
    def press_type(self) -> BUTTON_TYPES:
        return self._press_type

    def inc(self):
        self._duration += 1

    def __repr__(self):
        return f"{self.press_type.name} {self._duration}"

class BadgeBotApp(app.App):
    def __init__(self):
        self.button_states = Buttons(self)
        self.last_press: BUTTON_TYPES = BUTTON_TYPES["CANCEL"]

        self.scroll_offset = 0
        self.is_scroll = False
        self.instructions = []

        self.current_instruction = None
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
                self.is_scroll = not self.is_scroll
                self.button_states.clear()

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
            ctx.rgb(1,1,1).move_to(-80,-60).text("To Program:")
            ctx.rgb(1,1,0).move_to(-80,-30).text("Press C")
            ctx.rgb(1,1,1).move_to(-80, 10).text("When finished:")
            ctx.rgb(1,1,0).move_to(-80, 40).text("Long press C")
        elif self.current_state == BadgeBotAppState.RECEIVE_INSTR:
            ctx.rgb(1,1,0).move_to(-60,-60 + VERTICAL_OFFSET * (self.scroll_offset)).text("START")
            i_num = -1
            for i_num, instr in enumerate(self.instructions):
                ctx.rgb(1,1,0).move_to(-60,-60 + VERTICAL_OFFSET * (self.scroll_offset + i_num + 1)).text(repr(instr))
            ctx.rgb(1,1,0).move_to(-60,-60 + VERTICAL_OFFSET * (self.scroll_offset + i_num + 2)).text(repr(self.current_instruction))
            ctx.rgb(1,1,0).move_to(-60,-60 + VERTICAL_OFFSET * (self.scroll_offset + i_num + 3)).text("END")
        ctx.restore()

    def finalize_instruction(self):
        if self.current_instruction is not None:
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                self.scroll_offset -= 1
            self.current_instruction = None

__app_export__ = BadgeBotApp
