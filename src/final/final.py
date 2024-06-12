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

# Setup UART connection to arduino
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

# Threshold values
red_threshold = [(30, 55, 50, 65, 5, 55)] # [(30, 60, 35, 75, 5, 55)]
green_threshold = [(35, 60, -40, -5, -15, 10)] # [(25, 60, -35, -10, -15, 15)]

# ROI values
img = sensor.snapshot()
cubes_roi = (0, int(img.height() / 3) + 14, img.width(), int(img.height() * 2 / 3) - 14)

# Restrains values
cube_blob_size = 150
prop_thr = 0.7 # density >= 0.8 or solidity >= 1

# PID values
kp = 0.01
kd = 0.04 # 0.033
err_old = 0
offset = 80

last_cube_color = 'none'
avoidance_cooldown = 0.02

def clamp(val, min_intv, max_intv):
    if val < min_intv:
        return min_intv
    if val > max_intv:
        return max_intv
    return val

clock = time.clock()

while (True):
    clock.tick()
    img = sensor.snapshot()

    red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=cube_blob_size, area_threshold=cube_blob_size, merge=True)
    green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=cube_blob_size, area_threshold=cube_blob_size, merge=True)

    msg = "0\n"
    maxArea = 0
    color = 'none'
    saved_cube = None
    for blob in red_blobs:
        if blob.density() >= prop_thr and blob.area() > maxArea:
            maxArea = blob.area()
            saved_cube = blob
            color = 'red'
#        if blob.density() >= prop_thr:
#            img.draw_rectangle(blob.rect())
#            img.draw_cross(blob.cx(), blob.cy())
    for blob in green_blobs:
        if blob.density() >= prop_thr and blob.area() > maxArea:
            maxArea = blob.area()
            saved_cube = blob
            color = 'green'
#        if blob.density() >= prop_thr:
#            img.draw_rectangle(blob.rect())
#            img.draw_cross(blob.cx(), blob.cy())

    if last_cube_color != 'none' and last_cube_color != color:
        last_cube_color = color
        if uart.any() == 0:
            uart.write('a\n')
            print('a\n')

    if saved_cube != None:
        img.draw_rectangle(saved_cube.rect())
        img.draw_cross(saved_cube.cx(), saved_cube.cy())

        last_cube_color = color
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
    elif uart.any() == 0:
        uart.write('0\n')
        print('0\n')
