"""
Input label of this format:
    class x-centre y-centre width height
    
Do operations
    divide height by 2, and add to y-centre
    result is percentage, minus from 100
    percent of 640
Result is number of pixels below the bounding box

"""

from asyncore import read


def distance_to_sign(label):
    """
    Takes an input of YOLOv5 formatted label
    
    """
    with open(label) as f:
        l = read(l)
        