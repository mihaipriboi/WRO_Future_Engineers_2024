import sensor
import time
from pyb import UART, LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)

# UART 3, and baudrate.
uart = UART(3, 19200)

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

red_led.on()
blue_led.on()
time.sleep(1)
red_led.off()
blue_led.off()
time.sleep(1)

clock = time.clock()

# Define the LAB color threshold for black
black_threshold = [(0, 45, -10, 15, -25, 10)] # [(0, 30, -10, 10, -10, 10)]
blue_threshold = [(30, 75, 0, 25, -45, -15)]  # [(10, 80, -5, 25, -40, -10)]
orange_threshold = [(60, 85, 15, 50, -5, 50)] # [(50, 75, 15, 50, 10, 55)]

# ROI values
img = sensor.snapshot()
wall_roi = (0, int(img.height() / 3 + 10), img.width(), int(img.height() / 3))
lines_roi = (0, int(img.height() * 2 / 3 - 14), img.width(), int(img.height() / 3 + 14))

wall_height_threshold = 26
constant_height_time = 0.3

line_blob_size = 500
wall_blob_size = 3200

direction = 0
last_time_wall = time.time()

while (True):
    clock.tick()
    img = sensor.snapshot()

    wall_blobs = img.find_blobs(black_threshold, roi=wall_roi, pixels_threshold=wall_blob_size, area_threshold=wall_blob_size, merge=True)
    orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)
    blue_blobs = img.find_blobs(blue_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)

    if direction == 0:
        if len(orange_blobs) > 0:
            direction = 2
        elif len(blue_blobs) > 0:
            direction = 1

#    img.draw_rectangle(wall_roi, color=(255, 255, 0))

    msg = "0\n"
    time_now = -1
    avoid_cubes = True
    for blob in wall_blobs:
        # Draw a rectangle around each blob
#        wall_height = blob.rect()[3]
#        if wall_height > wall_height_threshold:
#            if time.time() - last_time > constant_height_time:
#                msg = "1\n"
#        else:
#            time_now = time.time()
#        img.draw_rectangle(blob.rect(), color=(255, 255, 255))
#        img.draw_cross(blob.cx(), blob.cy())

        if time.time() - last_time_wall > constant_height_time:
            msg = str(direction) + "\n"
        else:
            time_now = time.time()

    if time_now != -1:
        last_time_wall = time_now

    if uart.any() == 0:
        uart.write(msg)
        print(msg)
