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
#red_threshold = [(40, 55, 30, 60, 25, 60), (40, 55, 45, 70, 20, 65), (30, 55, 20, 65, -15, 50)]
red_threshold = [(30, 55, 20, 70, -15, 60)]

green_threshold = [(45, 90, -50, -10, -25, 20), (15, 60, -45, -15, -5, 20), (21, 50, -30, -12, -32, 12)]

#blue_threshold = [(10, 55, -15, 45, -45, -5)]
blue_threshold = [(10, 80, -5, 25, -50, -5)]

#orange_threshold = [(40, 80, 15, 50, 20, 75), (40, 85, -10, 40, 20, 80)]
#orange_threshold = [(50, 75, 5, 45, 15, 75)]
orange_threshold = [(50, 80, 5, 45, 15, 75)]
#orange_threshold = [(40, 85, -10, 40, 20, 80)]

#parking_threshold = [(25, 63, 45, 65, -10, 10)]
parking_threshold = [(30, 70, 10, 60, -15, 15)]

black_threshold = [(0, 45, -10, 15, -25, 10)]

# ROI values
img = sensor.snapshot()
cubes_roi = (0, int(img.height() / 2 + 8), img.width(), int(img.height() / 2 - 8))
lines_roi = (0, int(img.height() / 2 + 15), img.width(), int(img.height() / 3 + 15))
#parking_roi = (0, 56, img.width(), 20)
parking_roi = (0, int(img.height() / 2 + 8), img.width(), int(img.height() / 2 - 8))
wall_roi = (0, int(img.height() / 3 + 10), img.width(), int(img.height() * 2 / 3 - 10))
wall_roi_area = wall_roi[2] * wall_roi[3]

# restrains values
min_cube_height = 5
min_cube_size = 50
max_cube_size_red = 330 # 400
max_cube_size_green = 290 # 300
wall_blob_size = 4600

line_blob_size = 350
parking_blob_height_trigger = 10
parking_blob_size_trigger = 700
parking_blob_size_min = 35
density_thr = 0.7 # 0.6

# PID values
kp = 0.0033  # 0.0033
kd = 0.033   # 0.033
err_old = 0

# logic
direction = 0
FINAL = True

# force the val into the [min_intv, max_intv] interval
def clamp(val, min_intv, max_intv):
    if val < min_intv:
        return min_intv
    if val > max_intv:
        return max_intv
    return round(val, 3)

# detects if a blob is partially inside another blob
# by checking if the center of one blob is inside the minimum area rectangle that wraps the other blob
# giving the fact that this rectangle may be crooked, we have to use a special algorithm
# that is designed to check whether a point is inside a polygon or not based on the coordinates
def is_blob_in_blob(blob, blob2):
    if not blob2:
        return False
    polygon = blob2.min_corners()
    num_vertices = len(polygon)
    (x, y) = (blob.cx(), blob.cy())
    inside = False

    # store the first point in the polygon
    (p1x, p1y) = (polygon[0][0], polygon[0][1])

    # loop through each edge in the polygon
    for i in range(1, num_vertices + 1):
        # get the next point in the polygon
        (p2x, p2y) = (polygon[i % num_vertices][0], polygon[i % num_vertices][1])

        # check if the point is above the minimum y coordinate of the edge
        # check if the point is below the maximum y coordinate of the edge
        # check if the point is to the left of the maximum x coordinate of the edge
        if y > min(p1y, p2y) and y <= max(p1y, p2y) and x <= max(p1x, p2x):
            # calculate the x-intersection of the line connecting the point to the edge
            x_intersection = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x

            # check if the point is on the same line as the edge or to the left of the x-intersection
            if p1x == p2x or x <= x_intersection:
                # flip the inside flag
                inside = not inside

        # store the current point as the first point for the next iteration
        (p1x, p1y) = (p2x, p2y)

    # return the value of the inside flag
    return inside

# checks whether a blob is a cube or not based on the minimum height and density filters
# as well as another check:
# since in some light conditions the red the cube may be similar to the orange line or magenta parking walls
# and in some light conditions the green the cube may be similar to the blue line
# we have to check if the possible cube blob is inside any of these ones
# if it is, then it may just be a false alarm and we should ignore it
def is_cube(blob, line_blob, parking_blobs):
    if blob.density() >= density_thr and blob.h() > min_cube_height and not is_blob_in_blob(blob, line_blob):
        for parking_wall_blob in parking_blobs:
            if is_blob_in_blob(blob, parking_wall_blob):
                return False
        return True
    return False

# function that gets the biggest blob by size
def get_biggest_blob(blob_array):
    max_area = 0
    max_blob = None
    for blob in blob_array:
        # we're keeping the biggest parking blob
        if blob.area() > max_area:
            max_area = blob.area()
            max_blob = blob
    return max_blob

# checks if a parking wall blob meets the height and size requirements
def is_parking_wall(blob):
    if not blob:
        return False
    if blob.pixels() >= parking_blob_size_trigger and blob.area() >= parking_blob_size_trigger and blob.h() >= parking_blob_height_trigger:
        return True
    return False

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

    if FINAL: # if we're running the code for the final challenge
        # find the coloured blobs corresponding to the cubes
        red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
        green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size - 15, area_threshold=min_cube_size - 15, merge=True)

        # find the coloured blobs corresponding to the parking walls
        parking_blobs = img.find_blobs(parking_threshold, roi=parking_roi, pixels_threshold=parking_blob_size_min, area_threshold=parking_blob_size_min, merge=True)
        parking_wall_blob = get_biggest_blob(parking_blobs)
#        if parking_wall_blob:
#            img.draw_rectangle(parking_wall_blob.rect(), color=(0, 255, 0))

        # find the coloured blobs corresponding to the outside walls
        wall_blobs = img.find_blobs(black_threshold, roi=wall_roi, pixels_threshold=wall_blob_size, area_threshold=wall_blob_size, merge=True)
        outer_wall = get_biggest_blob(wall_blobs)
#        img.draw_rectangle(wall_roi, color=(0,0,255))
#        for blob in wall_blobs:
#            img.draw_rectangle(blob.rect(), color=(0, 255, 0))

        msg = "0\n"
        max_area = 0
        color = 'none'
        saved_cube = None
        for blob in red_blobs: # for every red blob
            # if they're passing the height and density filters
            # we're keeping the biggest one and its color
            if is_cube(blob, orange_blob, parking_blobs) and blob.area() > max_area:
                max_area = blob.area()
                saved_cube = blob
                color = 'red'
        for blob in green_blobs: # for every green blob
            # if they're passing the height and density filters
            # we're keeping the biggest one and its color
            if is_cube(blob, blue_blob, parking_blobs) and blob.area() > max_area:
                max_area = blob.area()
                saved_cube = blob
                color = 'green'

        if saved_cube != None: # if we saw a cube
#            img.draw_rectangle(saved_cube.rect())
#            img.draw_cross(saved_cube.cx(), saved_cube.cy())

            # if the cube area is over a certain threshold
            # it means we must avoid the cube as we are too close to it
            if (color == 'red' and saved_cube.pixels() >= max_cube_size_red) or (color == 'green' and saved_cube.pixels() >= max_cube_size_green):
#                img.draw_cross(saved_cube.cx(), saved_cube.cy(), color=(0, 255, 0))
                # send the right trigger
                if color == 'red':
                    uart.write('R\n')
#                    print('R\n')
                else:
                    uart.write('G\n')
#                    print('G\n')
                if has_line:
                    # if we must also turn, send the trigger
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
                uart.write(msg) # send the message
#                print(msg)
                if has_line:
                    # if we must also turn, send the turn trigger
                    uart.write(str(direction) + '\n')
#                    print(str(direction))
        elif has_line: # if we don't see any cubes
            # if we must turn, send the turn trigger
            uart.write(str(direction) + '\n')
#            print(str(direction))
        if is_parking_wall(parking_wall_blob):
            # if we saw the parking walls, send the parking trigger
#            img.draw_rectangle(parking_wall_blob.rect(), color=(0, 255, 0))
            uart.write('P\n')
#            print('P\n')
        if wall_blobs:
            # if the wall is big enough, send a slightly different message that helps us when parking
            # if not, send the classic one
            if outer_wall.pixels() >= wall_roi_area and outer_wall.area() >= wall_roi_area:
                uart.write('WP\n')
#                print('WP\n')
            else:
                uart.write('W\n')
#                print('W\n')
    else: # if we're running the quali code
        if has_line:
            # if we must turn, send the turn trigger
            uart.write(str(direction) + '\n')
#            print(str(direction))
