#util.py

from math import pi
from display import hexagon

def roundtext(ctx, t, r, top=False):
    ctx.save()
    h = ctx.font_size
    r=(h-r) if top else r-h/2
    w=sum(map(ctx.text_width, t))
    ctx.rotate(w/2/r)
    for c in t:
        w=ctx.text_width(c)
        ctx.rotate(-w/2/r)
        ctx.move_to(-w/2, r)
        ctx.text(c)
        ctx.move_to(0,0)
        ctx.rotate(-w/2/r)
    ctx.restore()

def draw_logo_animated(ctx, animation_counter=0, messages=None):
    legw = .12
    # 0:black, 1:white, 2:yellow
    colours = [(0, 0, 0), (1, 1, 1), (1.0, 0.84, 0)]

    #Hexagon
    rs = [(150, 1), (119, 0), (111, 2)]
    for r,c in rs:
        ctx.rgba(*colours[c], 1)
        hexagon(ctx, 0, 0, r)
    ctx.save()
    ctx.rotate(animation_counter * pi / 6)

    # Chip
    rs = [(45, 0), (38, 2), (33, 0)] # Outer, Inner, Solid
    # pins
    pin_width = rs[0][0] // 6
    ctx.rgba(0, 0, 0, 1)
    for j in range(5):
        ctx.rectangle(-rs[0][0]-pin_width, (1.75-j)*2*pin_width, 2*(rs[0][0]+pin_width), pin_width).fill()
        ctx.rectangle((1.75-j)*2*pin_width, -rs[0][0]-pin_width, pin_width, 2*(rs[0][0]+pin_width)).fill()
    # Chip Body    
    for r,c in rs:
        ctx.rgba(*colours[c], 1)
        ctx.round_rectangle(-r, -r, 2*r, 2*r, r//10).fill()
    ctx.restore()
    ctx.rgba(0,0,0,1)
    if messages is not None:
        if 0 < len(messages):
            roundtext(ctx,messages[0], 97, False)
        if 1 < len(messages):
            roundtext(ctx,messages[1], 97, True)

    
def chain(*iterables):
    for iterable in iterables:
        yield from iterable