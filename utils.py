#util.py

from math import pi


def roundtext(ctx, t, r, top=False, h=20):
    ctx.save()
    r=(h-r) if top else r
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
    #rs = [(150, 0), (100, 1), (75, 0), (45, 1), (40, 0)]
    rs = [(150, 0), (100, 1), (75, 0), (45, 0), (40, 0)]
    for r,c in rs:
        ctx.arc(0,0,r,0,2*pi,0)
        ctx.rgba(c,c,c, 1)
        ctx.fill()
    ctx.save()
    ctx.rotate(animation_counter * pi / 3)
    for i in range(6):
        ctx.begin_path()
        ctx.arc(0, 0, 105, i*pi*2/6, (i+legw)*pi*2/6, 0)
        ctx.arc(0, 0, 44, (i+legw)*pi*2/6, i*pi*2/6, -1)
        ctx.rgba(1, 1, 0, 1)
        ctx.fill()
    ctx.restore()
    #ps = [(-30, 5), (-20, 14), (-12, 5), (-5, 5), (5, 10), (12, 10), (20, 6)]
    #ctx.move_to(ps[0][0], ps[0][1])
    #ctx.begin_path()
    #for px, py in ps:
    #    ctx.line_to(px, py)
    #for px, py in ps:
    #    ctx.line_to(-px, -py)
    #ctx.line_to(-30, 5)
    #ctx.rgba(1, 1, 1, 1)
    #ctx.fill()
    ctx.rgba(0,0,0,1)
    if messages is not None:
        if 0 < len(messages):
            roundtext(ctx,messages[0], 97, False)
        if 1 < len(messages):
            roundtext(ctx,messages[1], 97, True)
    
def chain(*iterables):
    for iterable in iterables:
        yield from iterable