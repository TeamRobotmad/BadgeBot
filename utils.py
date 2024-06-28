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

def draw_logo_animated(ctx, rpm, animation_counter=0, messages=None, qr_code=None):
    # 0:black, 1:white, 2:yellow
    colours = [(0, 0, 0), (1, 1, 1), (1.0, 0.84, 0)]

    #Hexagon
    rs = [(150, 1), (120, 0), (114, 2)]
    for r,c in rs:
        ctx.rgba(*colours[c], 1)
        hexagon(ctx, 0, 0, r)
    
    ctx.save()
    ctx.rotate(rpm * animation_counter * pi / 30.0)
    # Chip
    rs = [(50, 0), (45, 2)] # , (40, 0)] # Outer, Inner, Solid (replaced by QR code)
    # pins - one rectangle does pins on opposite sides in one go
    pin_width = rs[0][0] // 6
    ctx.rgba(0, 0, 0, 1)
    for j in range(5):
        ctx.rectangle(-rs[0][0]-pin_width, (1.75-j)*2*pin_width, 2*(rs[0][0]+pin_width), pin_width).fill()
        ctx.rectangle((1.75-j)*2*pin_width, -rs[0][0]-pin_width, pin_width, 2*(rs[0][0]+pin_width)).fill()
    #Chip Body    
    for r,c in rs:
        ctx.rgba(*colours[c], 1)
        ctx.round_rectangle(-r, -r, 2*r, 2*r, r//10).fill()
    #QR Code        
    if qr_code is not None:
        draw_QRCode(ctx, qr_code, 2*rs[1][0], colours[2])
    ctx.restore()

    if messages is not None:
        ctx.rgba(0,0,0,1)
        if 0 < len(messages):
            roundtext(ctx,messages[0], 105, False)
        if 1 < len(messages):
            roundtext(ctx,messages[1], 105, True)

    

# QR code data
def draw_QRCode(ctx, qr_code, size=240, colour=(1,1,1)):
    qr_size = len( qr_code )
    
    #   Draw background - assume already drawn
    #ctx.rgb(*colour).rectangle(-size/2, -size/2, size, size).fill()

    #   Size of each QR code pixel on the canvas (enforce space for a border)
    pixel_size = int((size-8)//qr_size)

    #   Calculate the offset to start drawing the QR code (centre it within the available space)
    offset = -(pixel_size*qr_size//2)

    #   Loop through the array
    ctx.rgba(0,0,0,1)
    for row in range(qr_size):
        y = (row * pixel_size) + offset
        for col in range(qr_size):
            # check if the bit representing the col pixel is set
            # LSBit is on the left
            if qr_code[row] & (1 << col):
                x = (col * pixel_size) + offset
                ctx.rectangle(x, y, pixel_size+1, pixel_size).fill()  


def parse_version(version):
    #pre_components = ["final"]
    #build_components = ["0", "000000z"]
    #build = ""
    components = []
    if "+" in version:
        version, build = version.split("+", 1)
        #build_components = build.split(".")
    if "-" in version:
        version, pre_release = version.split("-", 1)
        #if pre_release.startswith("rc"):
        #    # Re-write rc as c, to support a1, b1, rc1, final ordering
        #    pre_release = pre_release[1:]
        #pre_components = pre_release.split(".")
    version = version.strip("v").split(".")
    components = [int(item) if item.isdigit() else item for item in version]
    #components.append([int(item) if item.isdigit() else item for item in pre_components])
    #components.append([int(item) if item.isdigit() else item for item in build_components])
    return components
    

def chain(*iterables):
    for iterable in iterables:
        yield from iterable