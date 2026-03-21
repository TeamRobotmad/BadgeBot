#utils.py
from math import pi

from display import hexagon


def roundtext(ctx, t, r, top=False):
    """Draw text t along a circular path with radius r.  If top is True, text is drawn above the center point, otherwise below."""
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
    """Draw the badge logo, with an animation based on the given RPM and animation counter.  Messages is an optional list of up to 2 strings to display below the logo.  QR code is an optional list of integers representing a QR code to display in the center of the logo."""
    # 0:black, 1:white, 2:yellow
    colours = [(0, 0, 0), (1, 1, 1), (1.0, 0.84, 0)]

    #Hexagon
    rs = [(150, 1), (120, 0), (114, 2)]
    for r,c in rs:
        ctx.rgba(*colours[c], 1)
        hexagon(ctx, 0, 0, r)
    
    ctx.save()
    ctx.rotate(rpm * animation_counter * pi / 30000.0)
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
def draw_QRCode(ctx, qr_code, size=240, colour=(1,1,1)):        # pylint: disable=unused-argument
    """Draw a QR code on the given canvas context.  qr_code is a list of integers, where each integer represents a row of the QR code, and the bits in the integer represent the pixels (LSBit is leftmost pixel).  Size is the total size of the area to draw within (including a border), and colour is the colour to use for the pixels (currently only black or white are supported)."""
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
    """Parse a version string into a list of components for comparison.  Components are converted to integers where possible, to allow correct ordering (e.g. 1.10 > 1.2).  Pre-release and build metadata are ignored for simplicity, as they are not currently used."""
    #pre_components = ["final"]
    #build_components = ["0", "000000z"]
    #build = ""
    components = []
    if "+" in version:
        version, build = version.split("+", 1)          # pylint: disable=unused-variable
        #build_components = build.split(".")
    if "-" in version:
        version, pre_release = version.split("-", 1)    # pylint: disable=unused-variable
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
    """Chain multiple iterables together into a single iterable."""
    for iterable in iterables:
        yield from iterable


# Value increment/decrement functions for positive integers only
def inc_value(v: int, l: int):
    """Increment the setting value.  If l > 0, increment by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.)"""
    if l==0:
        return v+1
    else:
        d = 10**l
        v = ((v // d) + 1) * d   # round up to the next multiple of 10^l
        return v


def dec_value(v: int, l: int):
    """Decrement the setting value.  If l > 0, decrement by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.)"""
    if l==0:
        return v-1
    else:
        d = 10**l
        v = (((v+(9*(10**(l-1)))) // d) - 1) * d   # round down to the next multiple of 10^l
        return v