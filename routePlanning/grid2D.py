from gridPoint import gridPoint
from settings import *
# THIS CLASS MIGHT BE COMPLETELY IRRELEVANT
# simply a 2D grid
class grid2D:
    def __init__(self, height):
        self.plane = [[gridPoint(j,i) for j in range(0, WIDTH)] for i in range(0, LENGTH)]
        self.height = height
        self.noFlyZonesIn = [] # all of the no flyzones that this 2d section cuts through