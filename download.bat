
REM mpy-cross -v app.py
mpy-cross -v linefollower.py
mpy-cross -v autotune.py
mpy-cross -v utils.py
REM mpremote cp app.mpy :apps/BadgeBot/app.mpy
REM mpremote cp utils.py :apps/BadgeBot/utils.mpy
mpremote cp LineFollower.mpy :apps/LineFollower/app.mpy
mpremote cp autotune.mpy :apps/LineFollower/autotune.mpy
mpremote cp utils.py :apps/LineFollower/utils.mpy