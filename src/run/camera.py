import sensor
import time
from pyb import UART, LED

# initialize the sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
#sensor.set_framerate(40)
sensor.set_vflip(True)
sensor.set_hmirror(True)

# disable auto gain, white balance, and exposure
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.set_auto_exposure(False, exposure_us=10000) # set constant exposure for the best visibility

# skip some frames to let the camera adjust
sensor.skip_frames(time=2000)

# setup UART connection to arduino
uart = UART(3, 19200)
# 3 - the uart config, meaning that we use pins P4 as the transmitter, P5 as the receiver
# 19200 - baud rate aka frequency, must match the one set up on the arduino

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

green_led.on()
blue_led.on()
time.sleep(0.5)
green_led.off()
blue_led.off()
time.sleep(0.5) # blink led with a cyan color so that we know the camera is ready

# threshold values

#red_threshold = [(35, 62, 40, 70, 5, 60)]
#red_threshold = [(40, 65, 30, 70, 20, 65)]
red_threshold = [(40, 55, 45, 70, 20, 65)]

green_threshold = [(45, 90, -50, -15, -20, 20), (15, 60, -45, -25, -5, 20), (21, 50, -30, -12, -32, 12)]
#blue_threshold = [(10, 55, -15, 45, -45, -5)]
blue_threshold = [(10, 80, -5, 25, -50, -5)]

#orange_threshold = [(40, 80, 15, 50, 20, 75), (40, 85, -10, 40, 20, 80)]
#orange_threshold = [(50, 75, 5, 45, 15, 75)]
orange_threshold = [(50, 80, 5, 45, 15, 75)]
#orange_threshold = [(40, 85, -10, 40, 20, 80)]

parking_threshold = [(25, 63, 45, 65, -10, 10)]

# ROI values
img = sensor.snapshot()
cubes_roi = (0, int(img.height() / 2 + 8), img.width(), int(img.height() / 2 - 8))
lines_roi = (0, int(img.height() / 2 + 15), img.width(), int(img.height() / 3 + 15))
parking_roi = (0, 56, img.width(), 20)

# restrains values
min_cube_height = 3
min_cube_size = 35
max_cube_size_red = 400 # 400
max_cube_size_green = 300 # 300

line_blob_size = 350
parking_blob_size = 1500
density_thr = 0.6 # 0.6

# PID values
kp = 0.0033
kd = 0.033
err_old = 0

# logic
direction = 0
final = True

# force the val into the [min_intv, max_intv] interval
def clamp(val, min_intv, max_intv):
    if val < min_intv:
        return min_intv
    if val > max_intv:
        return max_intv
    return round(val, 3)

# free the communication channel before sending a message
def flush_characters():
    while uart.any() != 0:
        uart.read()

clock = time.clock()

while (True):
    clock.tick()
    img = sensor.snapshot()

    # find the coloured blobs corresponding to the turn lines
    orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)
    blue_blobs = img.find_blobs(blue_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)

#    img.draw_rectangle(lines_roi, color=(0, 255, 0))

    orange_blob_w = None
    orange_blob_h = None
    max_width = 0
    max_height = 0
    for blob in orange_blobs:
        if blob.w() >= img.width() * 0.4: # if it meets the minimum width requirement
            if blob.w() > max_width: # if it's the biggest blob yet
                max_width = blob.w()
                orange_blob_w = blob # biggest blob on the width
        if blob.h() >= lines_roi[3] * 0.4: # if it meets the minimum height requirement
            if blob.h() > max_height: # if it's the biggest blob yet
                max_height = blob.h()
                orange_blob_h = blob # biggest blob on the height
    # if we have a blob meeting either the minimum width or height requirement we remember it
    orange_blob = orange_blob_w
    if not orange_blob:
        orange_blob = orange_blob_h

    blue_blob_w = None
    blue_blob_h = None
    max_width = 0
    max_height = 0
    for blob in blue_blobs:
        if blob.w() >= img.width() * 0.4: # if it meets the minimum width requirement
            if blob.w() > max_width: # if it's the biggest blob yet
                max_width = blob.w()
                blue_blob_w = blob # biggest blob on the width
        if blob.h() >= lines_roi[3] * 0.4: # if it meets the minimum height requirement
            if blob.h() > max_height: # if it's the biggest blob yet
                max_height = blob.h()
                blue_blob_h = blob # biggest blob on the height
    # if we have a blob meeting either the minimum width or height requirement we remember it
    blue_blob = blue_blob_w
    if not blue_blob:
        blue_blob = blue_blob_h

#    if orange_blob:
#        img.draw_edges(orange_blob.min_corners(), color=(255, 0, 0))
#        img.draw_line(orange_blob.major_axis_line(), color=(0, 255, 0))
#        img.draw_line(orange_blob.minor_axis_line(), color=(0, 255, 0))
#    if blue_blob:
#        img.draw_edges(blue_blob.min_corners(), color=(255, 0, 0))
#        img.draw_line(blue_blob.major_axis_line(), color=(0, 0, 255))
#        img.draw_line(blue_blob.minor_axis_line(), color=(0, 0, 255))

    if direction == 0: # if we didn't set a turn direction yet
        if orange_blob: # if the first line we saw was an orange one
            direction = 2
        elif blue_blob: # if the first line we saw was a blue one
            direction = 1

    has_line = False
    if orange_blob or blue_blob: # if we saw either coloured lines, we can make a turn
        has_line = True

    if final: # if we're running the code for the final challenge
        # find the coloured blobs corresponding to the cubes
        red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
        green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

        # find the coloured blobs corresponding to the parking walls
        parking_blobs = img.find_blobs(parking_threshold, roi=parking_roi, pixels_threshold=parking_blob_size, area_threshold=parking_blob_size, merge=True)

#        img.draw_rectangle(cubes_roi, color=(0,0,255))

        msg = "0\n"
        max_area = 0
        color = 'none'
        saved_cube = None
        for blob in red_blobs: # for every red blob
            # if they're passing the height and density filters
            # we're keeping the biggest one and its color
            if blob.density() >= density_thr and blob.h() > min_cube_height and blob.area() > max_area:
                max_area = blob.area()
                saved_cube = blob
                color = 'red'
#            if blob.density() >= density_thr:
#                img.draw_rectangle(blob.rect())
#                img.draw_cross(blob.cx(), blob.cy())
        for blob in green_blobs: # for every green blob
            # if they're passing the height and density filters
            # we're keeping the biggest one and its color
            if blob.density() >= density_thr and blob.h() > min_cube_height and blob.area() > max_area:
                max_area = blob.area()
                saved_cube = blob
                color = 'green'
#            if blob.density() >= density_thr:
#                img.draw_rectangle(blob.rect())
#                img.draw_cross(blob.cx(), blob.cy())

        if saved_cube != None: # if we saw a cube
#            img.draw_rectangle(saved_cube.rect())
#            img.draw_cross(saved_cube.cx(), saved_cube.cy())

            # if the cube area is over a certain threshold
            # it means we must avoid the cube as we are too close to it
            if (color == 'red' and saved_cube.pixels() >= max_cube_size_red) or (color == 'green' and saved_cube.pixels() >= max_cube_size_green):
#                img.draw_cross(saved_cube.cx(), saved_cube.cy(), color=(0, 255, 0))
                flush_characters() # making sure i can send the trigger
                # send the right trigger
                if color == 'red':
                    uart.write('R\n')
#                        print('R\n')
                else:
                    uart.write('G\n')
#                        print('G\n')
                if has_line: # maybe add centroid inclusion checker
                    # if we must also turn
                    while uart.any() != 0: # we make sure we can send the trigger
                        uart.read()
                    uart.write(str(direction) + '\n') # send the turn trigger
#                    print(str(direction))
            else: # if the cube isn't too big we must follow it
                # calculate the angle using PID
                err = saved_cube.cx() - img.width() / 2
                steering = err * kp + (err - err_old) * kd
                steering = -clamp(steering, -1, 1)
                err_old = err
                # craft the command
                if color == 'red':
                    msg = 'r' + str(steering) + '\n'
                else:
                    msg = 'g' + str(steering) + '\n'
                flush_characters() # making sure i can send the message
                uart.write(msg)
#                print(msg)
                if has_line: # maybe add centroid inclusion checker
                    # if we must also turn
                    flush_characters() # making sure i can send the turn trigger
                    uart.write(str(direction) + '\n')
#                    print(str(direction))
        elif has_line: # if we don't see any cubes
            # if we must turn
            flush_characters() # making sure i can send the turn trigger
            uart.write(str(direction) + '\n')
#            print(str(direction))
        if parking_blobs: # maybe add centroid inclusion checker
            # if we saw the parking walls
            flush_characters() # making sure i can send the parking trigger
            uart.write('P\n')
#            print('P\n')
    else: # if we're running the quali code
        if has_line:
            # if we must turn
            flush_characters() # making sure i can send the turn trigger
            uart.write(str(direction) + '\n')
#            print(str(direction))
