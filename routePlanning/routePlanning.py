# TO DO: incorporate a list of waypoints that have to be followed

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import Point
import shapely


import time

class routePlanning:

    def __init__(self):
        self.height = 100 # hard coded values for size of the grid
        self.width = 99
        self.grid = np.ones((self.height, self.width))
        self.noFlyZones = []
        self.softNoFlyZones = []
        self.allPathsAndDistances = []
        pass

    # generates a map with all of the noFlyZones and the flying zone
    # assume that theyre just given as a bunch of coordinates
    def createMap(self, flyZone, noFlyZones):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plt.imshow(self.grid, cmap = "binary")
        for zone in noFlyZones:
            # create a polygon from the no fly zone and display it as a patch
            zonePoly = Polygon(zone)
            self.noFlyZones.append(zonePoly)
            zonePatch = PolygonPatch(zonePoly)
            zonePatch.set_facecolor("orange")
            ax.add_patch(zonePatch)

            softZonePoly = shapely.affinity.scale(zonePoly, xfact = 1.3, yfact = 1.3, zfact = 1.3, origin = 'centroid') # create soft no fly zones by 
            self.softNoFlyZones.append(softZonePoly)
            softZonePatch = PolygonPatch(softZonePoly)
            softZonePatch.set_facecolor("orange")
            softZonePatch.set_alpha(0.4)
            ax.add_patch(softZonePatch)
        
        # show area which we CAN fly in with green
        self.flyZone = Polygon(flyZone)
        flyZonePatch = PolygonPatch(self.flyZone)
        flyZonePatch.set_facecolor("green")
        flyZonePatch.set_alpha(0.2)
        ax.add_patch(flyZonePatch)

        

    # greedyish method that tries to always move closer to endpoint if possible
    # startPoint and endPoint are tuples of floats representing coords in the form (x,y)
    def planRoute(self, startPoint, endPoint):
        # pre-process all points and check how far away they are from the endpoint
        # this block can be moved to init
        self.gridDistances = 2*max(self.height, self.width)*np.ones((self.height, self.width)) # initialise them as very far away
        for x in range(0, self.width):
            for y in range(0, self.height):
                self.gridDistances[y][x] = abs(endPoint[0] - x)**2 + abs(endPoint[1] - y) ** 2
        
        self.prevPositions = []
        self.prevPositions.append(startPoint)
        ret = self.getNextMove(endPoint)
        if ret == 0:
            print("No path could be found")
            return

        # now lets show the path
        xvals = [row[0] for row in self.prevPositions]
        yvals = [row[1] for row in self.prevPositions]
        print(xvals)
        print(yvals)
        plt.plot(xvals,yvals,'.-')
        plt.show()

    def getNextMove(self, endPoint):
        # check if current point is either endPoint or inside a polygon
        currPos = self.prevPositions[-1]
        if currPos == endPoint:
            return 1
        else:
            for noFlyZone in self.softNoFlyZones:
                if noFlyZone.contains(Point(currPos)):
                    del self.prevPositions[-1] # remove the previous addition to the list
                    return 0
            if not self.flyZone.contains(Point(currPos)):
                del self.prevPositions[-1] # remove the previous addition to the list
                return 0
        possibleMovesAndDistance = []
        # get the distance from end point 
        for x in range(currPos[0] - 1, currPos[0] + 2):
            for y in range(currPos[1] - 1, currPos[1] + 2): 
                if x == self.prevPositions[0] and y == self.prevPositions[1]:
                    continue
                possibleMovesAndDistance.append([(x,y), self.gridDistances[y][x]])
        possibleMovesAndDistance.sort(key = lambda x:x[1]) # sort in ascending order

        # do the move which will get us closer to the endpoint
        for move in possibleMovesAndDistance:
            self.prevPositions.append(move[0])
            ret = self.getNextMove(endPoint)
            if ret == 0: # if no path was found, then go to the next possible point
                continue 
            else:
                return 1

        return 0 # means no path was viable

if __name__ == "__main__":

    routePlanningObj = routePlanning()
    flyZone = [(1,1),(70,10), (80,80), (5,90)]
    noFlyZones = [
        [(5,5), (6,15), (20,20), (9,5)],
        [(40,40), (60,40), (60,60), (40,60)]

    ]
    routePlanningObj.createMap(flyZone, noFlyZones)
    routePlanningObj.planRoute((2,2), (79,79))


