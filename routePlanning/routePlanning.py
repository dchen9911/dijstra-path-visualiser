# TO DO: incorporate a list of waypoints that have to be followed
# Make it not recursion
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
import random
import math
from gridPoint import gridPoint

NO_FLY_ZONE_BOUNDARY_SIZE = 2
WIDTH = 1000
HEIGHT = 1000
POLYGON_NUMBER = 30
POLYGON_SIZE = 40

class routePlanning:
    def __init__(self):
        self.height = HEIGHT # hard coded values for size of the grid
        self.width = WIDTH
        self.grid = [[gridPoint(j,i) for j in range(0, WIDTH)] for i in range(0, HEIGHT)]
        self.noFlyZones = []
        self.softNoFlyZones = []
        self.allPathsAndDistances = []
        pass

    # generates a map with all of the noFlyZones and the flying zone
    # assume that theyre just given as a bunch of coordinates
    def createMap(self, flyZone, noFlyZones):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plt.imshow(np.ones((self.height, self.width)), cmap = "binary")
        for zone in noFlyZones:
            # create a polygon from the no fly zone and display it as a patch
            zonePoly = Polygon(zone)
            self.noFlyZones.append(zonePoly)
            zonePatch = PolygonPatch(zonePoly)
            zonePatch.set_facecolor("orange")
            ax.add_patch(zonePatch)

            # scaling based off size of desired boundary
            # possible to implement this based off oby.boudns
            radius = zonePoly.area ** 0.5
            scalingFact = (radius + NO_FLY_ZONE_BOUNDARY_SIZE*2)/radius
            softZonePoly = shapely.affinity.scale(zonePoly, xfact = scalingFact, yfact = scalingFact, zfact = scalingFact, origin = 'centroid') # create soft no fly zones by 
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
        #plt.show()

        

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
        count = 0
        
        # bulk of the path finding
        while True:
            if count > 20000: # some arbitrarily large number after which a solution probs doesnt exist
                print("no path found")
                return

            # visit the current point
            currPos = self.prevPositions[-1]
            currPoint = self.grid[currPos[1]][currPos[0]]
            currPoint.visit()

            continueFlag = False
            # check if we have reached the end yet or are in a no fly zone
            if currPos == endPoint:
                print("Path found")
                break
            else:
                for noFlyZone in self.softNoFlyZones:
                    if noFlyZone.contains(Point(currPos)):
                        del self.prevPositions[-1] # remove the previous addition to the list
                        continueFlag = True
                        break
                if not self.flyZone.contains(Point(currPos)):
                    del self.prevPositions[-1] # remove the previous addition to the list
                    continue
            if continueFlag:
                continue
            possibleMovesAndDistance = []
            # get the distance from end point 
            for x in range(currPos[0] - 1, currPos[0] + 2):
                for y in range(currPos[1] - 1, currPos[1] + 2): 
                    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT or self.grid[y][x].beenVisited(): # check if we've already visited or out of bounds
                        continue
                    possibleMovesAndDistance.append([(x,y), self.gridDistances[y][x]])
            
            if len(possibleMovesAndDistance) == 0: # this means there are no more possible moves
                print("No possible moves so backtrack?")
                del self.prevPositions[-1]
                continue

            possibleMovesAndDistance.sort(key = lambda x:x[1]) # sort in ascending order

            # add the one that has the shortest distance away from the endpoint
            self.prevPositions.append(possibleMovesAndDistance[0][0])

            count = count + 1
        print("count: " + str(count))
        # now lets show the path
        xvals = [row[0] for row in self.prevPositions]
        yvals = [row[1] for row in self.prevPositions]
        # print(xvals)
        # print(yvals)
        plt.plot(xvals,yvals,'.-')
        plt.show()
        return

        

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
                if (x,y) in self.prevPositions or x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT: # check if we've already visited or out of bounds
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
        del self.prevPositions[-1]
        return 0 # means no path was viable

def generateRandomNoFlyZones(n, size):
    zones = []
    for i in range(0,n):
        x = random.randrange(20,WIDTH)
        y = random.randrange(20,HEIGHT)
        nSides = np.random.normal(5, 1.5)
        nSides = round(nSides)
        if nSides < 3:
            nSides = 3
        thetaStep = 2*3.14159/nSides
        theta = np.random.normal(0, thetaStep/4)
        zone = []
        for j in range(0,nSides):
            r = np.random.normal(size, size/3)
            zone.append((x + r*math.cos(theta), y + r*math.sin(theta)))
            theta = theta + np.random.normal(thetaStep, thetaStep/4)
        zones.append(zone)
    return zones




if __name__ == "__main__":

    routePlanningObj = routePlanning()
    flyZone = [(1,1),(WIDTH,5), (WIDTH,HEIGHT), (5,HEIGHT)]
    noFlyZones = generateRandomNoFlyZones(POLYGON_NUMBER,POLYGON_SIZE)
    print(noFlyZones)
    #noFlyZones = [[(59.624414898112555, 81.40174607023728), (48.02314508819377, 86.59592378878118), (46.90299059872611, 71.39634435353503)], [(57.18027931634997, 38.29001668965869), (54.43530039767146, 45.63074538319422), (45.77066613734521, 42.756003445104355), (39.01808576676231, 44.92030294411203), (44.28385323609244, 40.08653747880057), (46.503672945158016, 36.38594001067716)], [(98.81836843413275, 72.93312811681439), (89.49513227037139, 78.85007976195699), (86.66596222903695, 66.90546112788763)], [(50.08481320568391, 35.72503611414005), (46.43885653343638, 39.31218583752805), (42.47651286590404, 32.59072582880941), (49.83985333606208, 32.69209160988164), (53.096603378125245, 35.713044556972235)], [(32.111308316264136, 40.172565718451814), (26.73136507047328, 42.835460210070885), (25.122587698977625, 42.703857181492054), (21.39047571307746, 46.172813386371416), (19.196345767308735, 43.05345996493098), (19.186660556856463, 35.237004304638816), (27.871139499357735, 32.43238753279634)]]
    routePlanningObj.createMap(flyZone, noFlyZones)
    routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3))

"""
    noFlyZones = [
        [(5,7), (6,17), (20,22), (9,7)],
        [(40,40), (60,40), (60,60), (40,60)],
        [(20,40), (25,40), (25,45),(20,45)],
        [(5,0), (10,0),(10,5),(5,5)],
        [(-6, 5), (-1,5), (-1,10),(-5,10)]
    ]
    """
    


