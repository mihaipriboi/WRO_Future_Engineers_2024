import sensor
import time
from pyb import UART, LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.set_framerate(40)
#sensor.set_auto_exposure(False, exposure_us=10000)

#sensor.set_contrast(3) # range -3 to +3
#sensor.set_brightness(3) # range -3 to +3
#sensor.set_saturation(3) # range -3 to +3

sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)

# Setup UART connection to arduino
uart = UART(3, 19200)

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

green_led.on()
blue_led.on()
time.sleep(0.5)
green_led.off()
blue_led.off()
time.sleep(0.5)

red_led.on()
green_led.on()
blue_led.on()

# Threshold values
red_threshold = [(25, 75, 40, 65, 20, 55)]
green_threshold = [(20, 55, -30, -5, -20, 20)]
black_threshold = [(0, 45, -10, 15, -25, 10)] # [(0, 30, -10, 10, -10, 10)]
blue_threshold = [(30, 75, 0, 25, -45, -15)]  # [(10, 80, -5, 25, -40, -10)]
orange_threshold = [(60, 85, 15, 50, -5, 50)] # [(50, 75, 15, 50, 10, 55)]

# ROI values
img = sensor.snapshot()
cubes_roi = (0, int(img.height() / 3) + 14, img.width(), int(img.height() * 2 / 3) - 14)
wall_roi = (0, int(img.height() / 3 + 10), img.width(), int(img.height() / 3))
lines_roi = (0, int(img.height() * 2 / 3 - 14), img.width(), int(img.height() / 3 + 14))

# Restrains values
min_cube_size = 150
max_cube_size = 900
line_blob_size = 500
wall_blob_size = 3200
prop_thr = 0.7 # density >= 0.8 or solidity >= 1

# PID values
kp = 0.01
kd = 0.032 # 0.033
err_old = 0
offset = 0

wall_height_threshold = 26
constant_height_time = 0.3

def clamp(val, min_intv, max_intv):
    if val < min_intv:
        return min_intv
    if val > max_intv:
        return max_intv
    return val

clock = time.clock()

direction = 0
last_time_wall = time.time()

while (True):
    clock.tick()
    img = sensor.snapshot()

    red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

    msg = "0\n"
    max_area = 0
    color = 'none'
    saved_cube = None
    for blob in red_blobs:
        if blob.density() >= prop_thr and blob.area() > max_area:
            max_area = blob.area()
            saved_cube = blob
            color = 'red'
#        if blob.density() >= prop_thr:
#            img.draw_rectangle(blob.rect())
#            img.draw_cross(blob.cx(), blob.cy())
    for blob in green_blobs:
        if blob.density() >= prop_thr and blob.area() > max_area:
            max_area = blob.area()
            saved_cube = blob
            color = 'green'
#        if blob.density() >= prop_thr:
#            img.draw_rectangle(blob.rect())
#            img.draw_cross(blob.cx(), blob.cy())

    if saved_cube != None:
        img.draw_rectangle(saved_cube.rect())
        img.draw_cross(saved_cube.cx(), saved_cube.cy())

        if saved_cube.pixels() >= max_cube_size:
            if uart.any() == 0:
                if color == 'red':
                    uart.write('R\n')
                    print('R\n')
                else:
                    uart.write('G\n')
                    print('G\n')
        else:
            if color == 'red':
                err = saved_cube.cx() - img.width() / 2 + offset
                steering = err * kp + (err - err_old) * kd
                steering = -clamp(steering, -1, 1)
                err_old = err
                msg = 'r' + str(steering) + '\n'
            else:
                err = saved_cube.cx() - img.width() / 2 - offset
                steering = err * kp + (err - err_old) * kd
                steering = -clamp(steering, -1, 1)
                err_old = err
                msg = 'g' + str(steering) + '\n'
            if uart.any() == 0:
                uart.write(msg)
                print(msg)
    else:
        wall_blobs = img.find_blobs(black_threshold, roi=wall_roi, pixels_threshold=wall_blob_size, area_threshold=wall_blob_size, merge=True)
        orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)
        blue_blobs = img.find_blobs(blue_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)

        if direction == 0:
            if len(orange_blobs) > 0:
                direction = 2
            elif len(blue_blobs) > 0:
                direction = 1

#        img.draw_rectangle(wall_roi, color=(255, 255, 0))

        msg = "0\n"
        time_now = -1
        avoid_cubes = True
        for blob in wall_blobs:
            # Draw a rectangle around each blob
#            wall_height = blob.rect()[3]
#            if wall_height > wall_height_threshold:
#                if time.time() - last_time > constant_height_time:
#                    msg = "1\n"
#            else:
#                time_now = time.time()
#            img.draw_rectangle(blob.rect(), color=(255, 255, 255))
#            img.draw_cross(blob.cx(), blob.cy())

            if time.time() - last_time_wall > constant_height_time:
                msg = str(direction) + "\n"
            else:
                time_now = time.time()

        if time_now != -1:
            last_time_wall = time_now

        if uart.any() == 0:
            uart.write(msg)
            print(msg)
