import asyncio
import app

from events.input import Buttons, BUTTON_TYPES

class BadgeBotApp(app.App):
    def __init__(self):
        self.button_states = Buttons(self)

def update(self, delta):
    if self.button_states.get(BUTTON_TYPES["CANCEL"]):
        # The button_states do not update while you are in the background.
        # Calling clear() ensures the next time you open the app, it stays open.
        # Without it the app would close again immediately.
        self.button_states.clear()
        self.minimise()

def draw(self, ctx):
    ctx.save()
    ctx.rgb(0.2,0,0).rectangle(-120,-120,240,240).fill()
    ctx.rgb(1,0,0).move_to(-80,0).text("UNDER CONSTRUCTION")
    ctx.restore()
