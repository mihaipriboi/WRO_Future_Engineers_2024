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

# logic
direction = 0
FINAL = True

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

if FINAL:
    green_led.on()
    blue_led.on() # blink led with a cyan color so that we know the camera is ready
    time.sleep(0.5)
else:
    green_led.on()
    red_led.on() # blink led with a yellow color so that we know the camera is ready
    time.sleep(0.5)

# turn led off
red_led.off()
green_led.off()
blue_led.off()
time.sleep(0.5)

# threshold values

#red_threshold = [(35, 62, 40, 70, 5, 60)]
#red_threshold = [(40, 65, 30, 70, 20, 65)]
#red_threshold = [(40, 55, 30, 60, 25, 60), (40, 55, 45, 70, 20, 65), (30, 55, 20, 65, -15, 50)]
#red_threshold = [(30, 55, 20, 70, -15, 60)]
red_threshold = [(18, 80, 5, 60, -10, 65)]

#green_threshold = [(45, 90, -50, -10, -25, 20), (15, 60, -45, -15, -5, 20), (21, 50, -30, -12, -32, 12)]
green_threshold = [(20, 85, -60, -5, -5, 55)]

#blue_threshold = [(10, 55, -15, 45, -45, -5)]
blue_threshold = [(10, 80, -5, 25, -50, -5)]

#orange_threshold = [(40, 80, 15, 50, 20, 75), (40, 85, -10, 40, 20, 80)]
#orange_threshold = [(50, 75, 5, 45, 15, 75)]
orange_threshold = [(50, 80, 5, 45, 15, 75)]
#orange_threshold = [(40, 85, -10, 40, 20, 80)]

#parking_threshold = [(25, 63, 45, 65, -10, 10)]
parking_threshold = [(25, 70, 14, 45, -15, 10)]

black_threshold = [(0, 45, -10, 15, -25, 10)]

# define threshold for "darkness"
dark_threshold = 50
darkness_limit = 90  # 90% threshold for too dark

# ROI values
img = sensor.snapshot()
cubes_roi = (0, int(img.height() / 2 + 8), img.width(), int(img.height() / 2 - 8))
lines_roi = (0, int(img.height() / 2 + 15), img.width(), int(img.height() / 3 + 15))
#parking_roi = (0, 56, img.width(), 20)
parking_roi = (0, int(img.height() / 2 + 8), img.width(), int(img.height() / 2 - 8))
wall_roi = (0, int(img.height() / 3 + 10), img.width(), int(img.height() * 2 / 3 - 10))
wall_roi_area = wall_roi[2] * wall_roi[3]

wall_left_roi = (0, 50, 60, 30)
wall_right_roi = (100, 50, 60, 30)

wall_dark_roi = (40, 50, 80, 40)

# restrains values
min_cube_height = 4
min_cube_size = 45
#max_cube_size_red = 390 # 400
#max_cube_size_green = 250 # 300
max_cube_height_red = 14
max_cube_height_green = 12
wall_blob_size = 4600

line_blob_size = 350
parking_blob_height_trigger = 10
parking_blob_size_trigger = 700
parking_blob_size_min = 35
density_thr = 0.7 # 0.6

# PID values
kp = 0.0019  # 0.0033
kd = 0.009   # 0.033
err_old = 0

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
    gray_img = img.copy()  # make a copy of the image
    gray_img.to_grayscale()  # convert to grayscale

    # determine the brightness of the ROIs to know if we're close to an outside wall or not
    left_dark_pixels = 0
    right_dark_pixels = 0
    middle_dark_pixels = 0

    # process pixels in the left ROI
    for y in range(wall_dark_roi[1], wall_dark_roi[1] + wall_dark_roi[3]):
        for x in range(wall_dark_roi[0], wall_dark_roi[0] + wall_dark_roi[2]):
            if gray_img.get_pixel(x, y) < dark_threshold:
                middle_dark_pixels += 1

    # process pixels in the left ROI
    for y in range(wall_left_roi[1], wall_left_roi[1] + wall_left_roi[3]):
        for x in range(wall_left_roi[0], wall_left_roi[0] + wall_left_roi[2]):
            if gray_img.get_pixel(x, y) < dark_threshold:
                left_dark_pixels += 1

    # process right ROI
    for y in range(wall_right_roi[1], wall_right_roi[1] + wall_right_roi[3]):
        for x in range(wall_right_roi[0], wall_right_roi[0] + wall_right_roi[2]):
            if gray_img.get_pixel(x, y) < dark_threshold:
                right_dark_pixels += 1


    # calculate darkness percentages
    left_total_pixels = wall_left_roi[2] * wall_left_roi[3]
    right_total_pixels = wall_right_roi[2] * wall_right_roi[3]
    middle_total_pixels = wall_dark_roi[2] * wall_dark_roi[3]

    left_dark_percentage = (left_dark_pixels / left_total_pixels) * 100
    right_dark_percentage = (right_dark_pixels / right_total_pixels) * 100
    middle_dark_percentage = (middle_dark_pixels / middle_total_pixels) * 100

#    img.draw_rectangle(wall_dark_roi, color=(0, 0, 255))  # Left ROI in blue
#    img.draw_string(wall_dark_roi[0], wall_dark_roi[1] - 10, str(middle_dark_percentage), color=(255, 0, 0))


#    # draw ROIs with blue outlines
#    img.draw_rectangle(wall_left_roi, color=(0, 0, 255))  # Left ROI in blue
#    img.draw_rectangle(wall_right_roi, color=(0, 0, 255))  # Right ROI in blue

#    # overlay the darkness percentages
#    img.draw_string(wall_left_roi[0], wall_left_roi[1] - 10, str(left_dark_percentage), color=(255, 0, 0))
#    img.draw_string(wall_right_roi[0], wall_right_roi[1] - 10, str(right_dark_percentage), color=(255, 0, 0))

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
        green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

        # find the coloured blobs corresponding to the parking walls
        parking_blobs = img.find_blobs(parking_threshold, roi=parking_roi, pixels_threshold=parking_blob_size_min, area_threshold=parking_blob_size_min, merge=False)
        parking_wall_blob = get_biggest_blob(parking_blobs)

#        if parking_wall_blob:
#            img.draw_rectangle(parking_wall_blob.rect(), color=(0, 0, 255))  # RGB: Blue
#            # draw a cross at the center of the detected object
#            img.draw_cross(parking_wall_blob.cx(), parking_wall_blob.cy(), color=(0, 0, 255))  # RGB: Blue
#            # add text annotation with information about the object
#            img.draw_string(parking_wall_blob.cx() - 47, parking_wall_blob.cy() - 48,
#                            "Object: Parking",
#                            color=(0, 255, 0), scale=1)  # Adjust scale as needed
#            img.draw_string(parking_wall_blob.cx() - 42, parking_wall_blob.cy() - 38,
#                            "Area: {}".format(parking_wall_blob.pixels()),
#                            color=(0, 255, 0), scale=1)  # Adjust scale as needed
#            img.draw_string(parking_wall_blob.cx() - 47, parking_wall_blob.cy() - 28,
#                            "X: {}, Y: {}".format(parking_wall_blob.cx(), parking_wall_blob.cy()),
#                            color=(0, 255, 0), scale=1)
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
            # draw a blue rectangle around the detected object
#            img.draw_rectangle(saved_cube.rect(), color=(0, 0, 255))  # RGB: Blue
#            # draw a cross at the center of the detected object
#            img.draw_cross(saved_cube.cx(), saved_cube.cy(), color=(0, 0, 255))  # RGB: Blue
#            # add text annotation with information about the object
#            img.draw_string(saved_cube.cx() - 47, saved_cube.cy() - 45,
#                            "Color: {}".format(color),
#                            color=(255, 0, 0), scale=1)  # Adjust scale as needed
#            img.draw_string(saved_cube.cx() - 42, saved_cube.cy() - 35,
#                            "Area: {}".format(saved_cube.pixels()),
#                            color=(255, 0, 0), scale=1)  # Adjust scale as needed
#            img.draw_string(saved_cube.cx() - 42, saved_cube.cy() - 55,
#                            "height: {}".format(saved_cube.h()),
#                            color=(255, 0, 0), scale=1)  # Adjust scale as needed
#            img.draw_string(saved_cube.cx() - 47, saved_cube.cy() - 25,
#                            "X: {}, Y: {}".format(saved_cube.cx(), saved_cube.cy()),
#                            color=(255, 0, 0), scale=1)  # Adjust scale as needed
#            img.draw_rectangle(saved_cube.rect())
#            img.draw_cross(saved_cube.cx(), saved_cube.cy())

            # if the cube area is over a certain threshold
            # it means we must avoid the cube as we are too close to it
            if (color == 'red' and saved_cube.h() >= max_cube_height_red) or (color == 'green' and saved_cube.h() >= max_cube_height_green):
#                img.draw_cross(saved_cube.cx(), saved_cube.cy(), color=(0, 255, 0))
                # send the right trigger
                err_old = 0
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

                if color == 'red':
                    err = saved_cube.cx() - (img.width() / 2)
                else:
                    err = saved_cube.cx() - (img.width() / 2 - 5)


#                if left_dark_percentage > darkness_limit and saved_cube.pixels() <= 250:
#                    steering = -0.35 # too dark on the left, steer right
##                    print("debug")
#                elif right_dark_percentage > darkness_limit and saved_cube.pixels() <= 250:
#                    steering = 0.35
##                    print("debug")
#                else:
                steering = err * kp + (err - err_old) * kd
                steering = -clamp(steering, -1, 1)
                err_old = err
                # craft the command
                # adjust error based on darkness levels
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
            img.draw_rectangle(parking_wall_blob.rect(), color=(0, 255, 0))
            uart.write('P\n')
#            print('P\n')
        if wall_blobs:
            # if the wall is big enough, send a slightly different message that helps us when parking
            # if not, send the classic one
            if middle_dark_percentage > 75:
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
