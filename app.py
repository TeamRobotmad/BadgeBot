import asyncio
import app

from events.input import Buttons, BUTTON_TYPES

class BadgeBotApp(app.App):
    def __init__(self):
        self.button_states = Buttons(self)
        self._r_count = 0
        self._l_count = 0

    def update(self, delta):
        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            # The button_states do not update while you are in the background.
            # Calling clear() ensures the next time you open the app, it stays open.
            # Without it the app would close again immediately.
            self.button_states.clear()
            self.minimise()

        if self.button_states.get(BUTTON_TYPES["RIGHT"]):
            self.button_states.clear()
            self._r_count += 1

        if self.button_states.get(BUTTON_TYPES["LEFT"]):
            self.button_states.clear()
            self._l_count += 1

    def draw(self, ctx):
        ctx.save()
        ctx.rgb(0.2,0,0).rectangle(-120,-120,240,240).fill()
        ctx.rgb(1,1,1).move_to(-100,-20).text("Hi Chog")
        ctx.rgb(1,1,0).move_to(-100,20).text("Right press " + str(self._r_count))
        ctx.rgb(1,0,1).move_to(-100,60).text("Left press " + str(self._l_count))
        ctx.restore()
