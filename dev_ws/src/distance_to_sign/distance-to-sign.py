"""
Input label with the YOLOv5 format:
    class x-centre y-centre width height
    Return height of box (after 4th space)
Do operations
    divide height by 2, and add to y_centre
    result is percentage, minus from 100
    percent of 640
Result is number of pixels below the bounding box
Calibrate
    48cm is 636.5101500000001px
    54cm is 637.2563500000001px
    Linear equations found
        48 = m(636.5101500000001) + c
        54 = m(637.2563500000001) + c
    Solved linear equation (rounded inputs to 2 decimals)
        y = 8x - 5044.08
! THIS IS SO COOL IM ACTUALLY USING MATH IN A PRACTICAL ENVIRONMENT
"""
import pathlib

def calibration(label):
    with open(label) as f:
        split = f.read().split()
        # height = split[4]
        # y-centre = split[2]
        # percent = 100 - (split[4] / 2 + split[2])
        pixels = 640 / 100 * (100 - (float(split[4]) / 2 + float(split[2])))
        print(pixels)

def distance_to_sign(label):
    with open(label)

calibration("54.txt")