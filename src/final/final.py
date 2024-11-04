import sensor
import time
from pyb import UART, LED

# Initialize the sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
#sensor.set_framerate(40)
sensor.set_vflip(True)
sensor.set_hmirror(True)

# Disable auto gain, white balance, and exposure
sensor.set_auto_gain(False)  # Must be turned off for color tracking
sensor.set_auto_whitebal(False)  # Must be turned off for color tracking
sensor.set_auto_exposure(False, exposure_us=10000)

# Skip some frames to let the camera adjust
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

#red_led.on()
#green_led.on()
#blue_led.on()

# Threshold values

#red_threshold = [(35, 62, 40, 70, 5, 60)]
#red_threshold = [(40, 65, 30, 70, 20, 65)]
red_threshold = [(40, 55, 45, 70, 20, 65)]

green_threshold = [(45, 90, -50, -15, -20, 20), (21, 50, -30, -12, -32, 12)]
#blue_threshold = [(10, 55, -15, 45, -45, -5)]
blue_threshold = [(10, 80, -5, 25, -50, -5)]

#orange_threshold = [(40, 80, 15, 50, 20, 75), (40, 85, -10, 40, 20, 80)]
#orange_threshold = [(50, 75, 5, 45, 15, 75)]
orange_threshold = [(50, 80, 5, 45, 15, 75)]
#orange_threshold = [(40, 85, -10, 40, 20, 80)]

# ROI values
img = sensor.snapshot()
cubes_roi = (0, int(img.height() / 2 + 8), img.width(), int(img.height() / 2 - 8))
lines_roi = (0, int(img.height() / 2 + 15), img.width(), int(img.height() / 3 + 15))

# Restrains values
min_cube_height = 3
min_cube_size = 35
max_cube_size_red = 400 # 450
max_cube_size_green = 350 # 600

line_blob_size = 350
density_thr = 0.6 # density >= 0.8 or solidity >= 1

# PID values
kp = 0.0033
kd = 0.033 # 0.033
err_old = 0

# Logic
direction = 0
final = True

def clamp(val, min_intv, max_intv):
    if val < min_intv:
        return min_intv
    if val > max_intv:
        return max_intv
    return round(val, 3)

clock = time.clock()

while (True):
    clock.tick()
    img = sensor.snapshot()

    orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)
    blue_blobs = img.find_blobs(blue_threshold, roi=lines_roi, pixels_threshold=line_blob_size, area_threshold=line_blob_size, merge=True)

    has_line = False

#    img.draw_rectangle(lines_roi, color=(0, 255, 0))

#    test = img.find_rects(roi=lines_roi, threshold=0)
#    for rect in test:
#        print(rect)
#        img.draw_rectangle(rect[:4], color=(255, 0, 0))

    orange_blob_w = None
    orange_blob_h = None
    max_width = 0
    max_height = 0
    for blob in orange_blobs:
        if blob.w() >= img.width() * 0.4:
            if blob.w() > max_width:
                max_width = blob.w()
                orange_blob_w = blob
        if blob.h() >= lines_roi[3] * 0.4:
            if blob.h() > max_height:
                max_height = blob.h()
                orange_blob_h = blob
    orange_blob = orange_blob_w
    if not orange_blob:
        orange_blob = orange_blob_h

    blue_blob_w = None
    blue_blob_h = None
    max_width = 0
    max_height = 0
    for blob in blue_blobs:
        if blob.w() >= img.width() * 0.4:
            if blob.w() > max_width:
                max_width = blob.w()
                blue_blob_w = blob
        if blob.h() >= lines_roi[3] * 0.4:
            if blob.h() > max_height:
                max_height = blob.h()
                blue_blob_h = blob
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

    if direction == 0:
        if orange_blob:
            direction = 2
        elif blue_blob:
            direction = 1

    if orange_blob and direction == 2:
        has_line = True
    elif blue_blob and direction == 1:
        has_line = True
#    if orange_blob or blue_bob:
#        has_line = True

    if final:
        red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
        green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

#        img.draw_rectangle(cubes_roi, color=(0,0,255))

        msg = "0\n"
        max_area = 0
        color = 'none'
        saved_cube = None
        for blob in red_blobs:
            if blob.density() >= density_thr and blob.h() > min_cube_height and blob.area() > max_area:
                max_area = blob.area()
                saved_cube = blob
                color = 'red'
#            if blob.density() >= density_thr:
#                img.draw_rectangle(blob.rect())
#                img.draw_cross(blob.cx(), blob.cy())
        for blob in green_blobs:
            if blob.density() >= density_thr and blob.h() > min_cube_height and blob.area() > max_area:
                max_area = blob.area()
                saved_cube = blob
                color = 'green'
#            if blob.density() >= density_thr:
#                img.draw_rectangle(blob.rect())
#                img.draw_cross(blob.cx(), blob.cy())

        if saved_cube != None:
#            img.draw_rectangle(saved_cube.rect())
#            img.draw_cross(saved_cube.cx(), saved_cube.cy())

            if (color == 'red' and saved_cube.pixels() >= max_cube_size_red) or (color == 'green' and saved_cube.pixels() >= max_cube_size_green):
#                img.draw_cross(saved_cube.cx(), saved_cube.cy(), color=(0, 255, 0))
                if uart.any() == 0:
                    if color == 'red':
                        uart.write('R\n')
#                        print('R\n')
                    else:
                        uart.write('G\n')
#                        print('G\n')
                    if has_line: # maybe add centroid inclusion checker
                        while uart.any() != 0:
                            time.sleep_ms(0)
                        uart.write(str(direction) + '\n')
#                        print(str(direction))
            else:
                err = saved_cube.cx() - img.width() / 2
                steering = err * kp + (err - err_old) * kd
                steering = -clamp(steering, -1, 1)
                err_old = err
                if color == 'red':
                    msg = 'r' + str(steering) + '\n'
                else:
                    msg = 'g' + str(steering) + '\n'
                if uart.any() == 0:
                    uart.write(msg)
#                    print(msg)
                    if has_line: # maybe add centroid inclusion checker
                        while uart.any() != 0:
                            time.sleep_ms(0)
                        uart.write(str(direction) + '\n')
#                        print(str(direction))
        elif has_line and uart.any() == 0:
            uart.write(str(direction) + '\n')
#            print(str(direction))
    else:
        if has_line and uart.any() == 0:
            uart.write(str(direction) + '\n')
#            print(str(direction))
