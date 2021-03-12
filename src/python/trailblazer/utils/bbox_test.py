#! /usr/bin/python

import os
import pygame
import sys
import time
from math import cos, sin, atan, sqrt, pi, hypot, radians

# This is simply a way of verifying that bbox being drawn are accurate.
# Double checking the math. 
# Not used in scenario generation.

# Return the top left and bottom right point of required bbox to contain a rectangle 
# of length 'l' and width 'w' at angle 'angle' in clockwise degrees from vertical
# 12 o'clock (navigational standard)
# NOTE: The accuracy of these bounding boxes needs to be verified
def calc_bbox_xy(x, y, w, l, angle):
    pygame.init()
    w, l, angle = map(float, [w, l , angle])
    x, y = map(int, [x, y])
    screen = pygame.display.set_mode((640, 480))
    screen.fill((255,255,255))
    rad = radians(angle)
    hyp = hypot(w, l)
    pygame.draw.circle(screen, (255, 0, 0), (x, 480 - y), 5)
    if angle <= 90: # First quad
        bbox_w = abs(hyp*(sin(rad + atan(w/l))))
        bbox_h = abs(hyp*(cos(rad - atan(w/l))))
        TL_x = x - w*cos(rad)/2 - l*sin(rad)
        TL_y = y + w*sin(rad)/2
        tl_x = x - w*cos(rad)/2
        tl_y = TL_y
        tr_x = x + w*cos(rad)/2
        tr_y = y - w*sin(rad)/2
        bl_x = TL_x
        bl_y = TL_y - l*cos(rad)
        br_x = tr_x - l*sin(rad)
        br_y = tr_y - l*cos(rad)
    elif angle <= 180: # Second quad
        bbox_w = abs(hyp*(sin(rad - atan(w/l))))
        bbox_h = abs(hyp*(cos(rad + atan(w/l))))
        TL_x = x + w*cos(pi - rad)/2 - bbox_w
        TL_y = y - w*sin(pi - rad)/2 + bbox_h
        tl_x = TL_x
        tl_y = TL_y - w*sin(pi - rad)
        tr_x = TL_x + w*cos(pi - rad)
        tr_y = TL_y
        bl_x = x - w*cos(pi - rad)/2
        bl_y = TL_y - bbox_h
        br_x = TL_x + bbox_w
        br_y = y + w*sin(pi - rad)/2
    elif angle <= 270: # Third quad
        bbox_w = abs(hyp*(sin(rad + atan(w/l))))
        bbox_h = abs(hyp*cos(rad - atan(w/l)))
        TL_x = x - w*cos(rad - pi)/2
        TL_y = y - w*sin(rad - pi)/2 + bbox_h
        tl_x = TL_x + l*cos(3*pi/2 - rad)
        tl_y = TL_y
        tr_x = TL_x + bbox_w
        tr_y = TL_y - w*sin(rad - pi)
        bl_x = TL_x
        bl_y = y + w*sin(rad - pi)/2
        br_x = x + w*cos(rad - pi)/2
        br_y = TL_y - bbox_h
    else: # Fourth quad
        bbox_w = abs(hyp*(sin(rad - atan(w/l))))
        bbox_h = abs(hyp*(cos(rad + atan(w/l))))
        TL_x = x - abs(w*sin(rad - 3*pi/2)/2)
        TL_y = y + abs(w*cos(rad - 3*pi/2)/2)
        tl_x = TL_x + bbox_w
        tl_y = TL_y - l*cos(2*pi - rad)
        tr_x = TL_x + l*sin(2*pi - rad)
        tr_y = TL_y - bbox_h
        bl_x = TL_x + w*sin(rad - 3*pi/2)
        bl_y = TL_y
        br_x = TL_x
        br_y = TL_y - w*cos(rad - 3*pi/2)

    assert bbox_w >= w - 0.1, "Math error, bounding box width is unrealistically small."
    assert bbox_h >= w - 0.1, "Math error, bounding box height is unrealistically small."

    hyp_r = hypot(br_x - tl_x, tl_y - br_y)
    hyp_l = hypot(bl_x - tr_x, tr_y - bl_y)
    print("True big rect hypot: %s" % hypot(bbox_w, bbox_h))
    print("True small rect hypot: %s" % hypot(w, l))
    print("Calc hypot: %s" % hyp_r, hyp_l)
    pygame.draw.rect(screen, (0, 255, 0), pygame.Rect(TL_x, 480 - TL_y, bbox_w, bbox_h), 2)
    pygame.draw.lines(screen, (0, 0, 255), True, [(tl_x, 480 - tl_y), (tr_x, 480 - tr_y),\
            (br_x, 480 - br_y), (bl_x, 480 - bl_y)])
    pygame.display.flip()

    BR_x = TL_x + bbox_w
    BR_y = TL_y - bbox_h
    return TL_x, TL_y, BR_x, BR_y

if __name__ == '__main__':
    args = sys.argv[1:]
    calc_bbox_xy(args[0], args[1], args[2], args[3], args[4])

    time.sleep(30)
