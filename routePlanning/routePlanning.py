# TO DO: incorporate a list of waypoints that have to be followed
# TO DO: linearly interpolate the path so less way points are given to the drone
# TO DO: Check if line segments cross no fly zones or not
# TO DO: make geofence smaller

from settings import *

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import shapely
import time
import random
import math
import sys
import argparse
import math

from minHeap import MinHeap
from matplotlib.path import Path
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import Point

from gridPoint import gridPoint

class routePlanning:
    def __init__(self, flyZone, noFlyZone):
        self.height = HEIGHT # hard coded values for size of the grid
        self.width = WIDTH
        self.grid = [[gridPoint(j,i) for j in range(0, WIDTH)] for i in range(0, HEIGHT)]
        self.noFlyZones = []
        self.softNoFlyZones = []
        self.flyZone = Polygon(flyZone)
        for zone in noFlyZones:
            # create a polygon from the no fly zone and display it as a patch
            zonePoly = Polygon(zone)
            self.noFlyZones.append(zonePoly)

            # scaling based off size of desired boundary
            # possible to implement this based off obj.bounds
            radius = zonePoly.area ** 0.5
            scalingFact = (radius + NO_FLY_ZONE_BOUNDARY_SIZE*2)/radius
            softZonePoly = shapely.affinity.scale(zonePoly, xfact = scalingFact, yfact = scalingFact, zfact = scalingFact, origin = 'centroid') # create soft no fly zones by making polygons bigger
            self.softNoFlyZones.append(softZonePoly)

        # efficiently preprocess points by going through each polygon and looking at each points within its bounds
        for noFlyZone in self.softNoFlyZones:
            (minx, miny, maxx, maxy) = noFlyZone.bounds
            for x in range (math.floor(minx), math.ceil(maxx) + 1):
                for y in range (math.floor(miny), math.ceil(maxy) + 1):
                    # check out of bounds
                    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
                        continue
                    point = Point(x,y)
                    if noFlyZone.contains(point):
                        self.grid[y][x].setNotAccessible()
            
        for x in range(0, WIDTH):
            for y in range(0, HEIGHT):
                if not self.grid[y][x].isAccessible(): # slightly more efficient
                    continue
                point = Point(x,y)
                if not self.flyZone.contains(point): # if not within geofence, then also make it inaccessible
                    print("oi run")
                    self.grid[y][x].setNotAccessible()
        print("Finished preprocessing")
        

    # generates a map with all of the noFlyZones and the flying zone
    # assume that theyre just given as a bunch of coordinates
    def createMap(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plt.imshow(np.ones((self.height, self.width)), cmap = "binary")
        for zonePoly in self.noFlyZones:
            # create a polygon from the no fly zone and display it as a patch
            zonePatch = PolygonPatch(zonePoly)
            zonePatch.set_facecolor("orange")
            ax.add_patch(zonePatch)

        for softZonePoly in self.softNoFlyZones:
            softZonePatch = PolygonPatch(softZonePoly)
            softZonePatch.set_facecolor("orange")
            softZonePatch.set_alpha(0.4)
            ax.add_patch(softZonePatch)
        
        # show area which we CAN fly in with green
        flyZonePatch = PolygonPatch(self.flyZone)
        flyZonePatch.set_facecolor("green")
        flyZonePatch.set_alpha(0.2)
        ax.add_patch(flyZonePatch)
        
        # now lets show the path
        xvals = [row[0] for row in self.prevPositions]
        yvals = [row[1] for row in self.prevPositions]

        plt.plot(xvals,yvals,'.-')
        plt.show()

    # greedyish method that tries to always move closer to endpoint if possible
    # startPoint and endPoint are tuples of floats representing coords in the form (x,y)
    def planRoute(self, startPoint, endPoint, method = 'g'):
        # pre-process all points and check how far away they are from the endpoint
        # this block can be moved to init
        self.gridDistances = 2*max(self.height, self.width)*np.ones((self.height, self.width)) # initialise them as very far away
        for x in range(0, self.width):
            for y in range(0, self.height):
                self.gridDistances[y][x] = abs(endPoint[0] - x)**2 + abs(endPoint[1] - y) ** 2
        
        self.prevPositions = []
        self.prevPositions.append(startPoint)
        if method == 'd':
            self.dijkstraFindPath(endPoint)
        else:
            self.greedyFindPath(endPoint)
        return

    def dijkstraFindPath(self, endPoint):
        startPoint = self.prevPositions[-1]
        self.grid[startPoint[1]][startPoint[0]].setDistance(0)
        pq = MinHeap()
        # create the priority queue
        for x in range(0, WIDTH):
            for y in range (0, HEIGHT):
                self.grid[y][x].setDistance(self.grid[y][x].getDistance())
                pq.insert(self.grid[y][x], self.grid[y][x].getDistance())
        count = 0
        while pq.counter != 0:
            currGridPoint = pq.del_min()
            if currGridPoint.getDistance() > LARGE_DISTANCE: # these are the points that cannot be reached from the start point
                print("points covered: " + str(count))
                break
            currGridPoint.visit()
            currX = currGridPoint.x
            currY = currGridPoint.y

            # go through all the neighbours of the current point
            for x in range(currX - 2, currX + 3):
                for y in range(currY - 2, currY + 3): 
                    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT : # check if out of bounds
                        continue
                    # check if point has been visited and whether it is accessible or not
                    currPoint = self.grid[y][x]
                    if currPoint.beenVisited() or not currPoint.isAccessible():
                        continue
                    
                    oldDist = currPoint.getDistance()
                    newDist = currGridPoint.getDistance() + (abs(x - currX)**2 + abs(y - currY)**2) ** 0.5
                    if newDist < oldDist:
                        currPoint.setDistance(newDist)
                        pq.insert(currPoint, currPoint.getDistance())
                        currPoint.setParent(currGridPoint)
            count = count + 1

        endGridPoint = self.grid[endPoint[1]][endPoint[0]]
        if endGridPoint.getDistance() > LARGE_DISTANCE:
            print("no path could be found")
            return
        startGridPoint = self.grid[startPoint[1]][startPoint[0]]

        while endGridPoint != startGridPoint:
            self.prevPositions.append((endGridPoint.x, endGridPoint.y))
            endGridPoint = endGridPoint.getParent()
        del self.prevPositions[0]
        self.prevPositions.append(startPoint)

        

    def greedyFindPath(self,endPoint):
        count = 0
            
        while True:
            if count > 20000 or len(self.prevPositions) == 0: # some arbitrarily large number after which a solution probs doesnt exist OR we backtrack back to the beginning with no options
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
                # check if point has been visited and it is accessible or not
                if not currPoint.isAccessible():
                    del self.prevPositions[-1] # remove the previous addition to the list
                    continue

            possibleMovesAndDistance = []
            # get the distance from end point 
            for x in range(currPos[0] - 3, currPos[0] + 4):
                for y in range(currPos[1] - 3, currPos[1] + 4): 
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
        print("number of steps: " + str(count))
        return

def generateRandomNoFlyZones(n, size):
    zones = []
    for i in range(0,n):
        x = random.randrange(5,WIDTH-5)
        y = random.randrange(5,HEIGHT-5 )
        if x < POLYGON_SIZE*2 and y < POLYGON_SIZE*2 or x > WIDTH - POLYGON_SIZE*2 and y > HEIGHT - POLYGON_SIZE*2:
            i = i - 1
            continue
        nSides = np.random.normal(5.5, 1.5)
        nSides = round(nSides)
        if nSides < 3:
            nSides = 3
        thetaStep = 2*3.14159/nSides
        theta = np.random.normal(0, thetaStep/4)
        zone = []
        for j in range(0,nSides):
            r = np.random.normal(size, size/2)
            zone.append((x + r*math.cos(theta), y + r*math.sin(theta)))
            theta = theta + np.random.normal(thetaStep, thetaStep/4)
        zones.append(zone)
    return zones

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plans the route')
    parser.add_argument('-d', '--dijkstra', required=False, help='Use dijkstra rather than greedy', action='store_true')
    args = vars(parser.parse_args())

    flyZone = [(1,1),(WIDTH,5), (WIDTH,HEIGHT), (5,HEIGHT)]
    #noFlyZones = generateRandomNoFlyZones(POLYGON_NUMBER,POLYGON_SIZE)
    #print(noFlyZones)
    noFlyZones = [[(53.92467898230293, 50.905121078506966), (52.62260758551787, 62.8942659373792), (46.27618221864204, 50.5497070023759), (47.48715243253119, 37.136872248352006), (64.21489658733563, 44.210027470531855)], [(65.25735220121798, 13.913370847904874), (54.61252076809604, 20.968176814070276), (42.96320025094971, 25.342306457861408), (41.58939141413785, 14.388539308474764), (46.59262368088967, 6.76622804823079), (54.997070889985196, 6.4075630571338)], [(87.98294335977342, 90.32079853159945), (73.09271456833231, 97.06991841530086), (63.21400994225273, 93.86106070652957), (68.81839996496718, 81.94950909550245), (80.25719865739299, 85.13585051458573)], [(100.16793582693788, 6.909950658441927), (94.11255207263643, 14.073410561444186), (83.16509823497191, 9.183700089833671), (87.10561079823182, 0.34042690013851384), (97.70372234976503, -3.4622141761992804)], [(40.41349153246735, 73.08783428444481), (31.377286938812194, 80.08299204179757), (42.11259906295611, 66.73805329739263)], [(83.24132227357995, 57.568645858900744), (72.64614509242696, 62.87038206802974), (61.45091110437718, 58.04031902840369), (72.12025492378133, 50.98289651424176), (75.54969498942043, 40.4679761563251), (84.14361862582714, 48.46278723194654)], [(57.173369646820454, 68.86794780234784), (47.11264226072787, 72.96716947966489), (30.804096694160037, 74.51695033728022), (38.117464885810456, 59.26359176212536), (56.21037355078592, 64.45422163076718)], [(37.12212981830822, 92.75331799803605), (33.958459745116265, 100.90538106829243), (20.148996615241686, 106.94000306447127), (24.694711521547422, 92.58681986260963), (25.578421665631417, 80.0157776056027), (37.23182801526152, 81.1577145222592), (37.07809013221568, 90.20662125867214)], [(57.304392184911165, 27.338005657390937), (58.00137229578584, 39.22379057673793), (41.70029192591472, 42.34401430263042), (45.56951842198082, 26.340248872787015)], [(25.164671486032965, 28.819996909323343), (23.503676615376655, 44.43445709172494), (9.50814498192786, 41.92999015135076), (16.05669735556644, 29.69883177869935), (10.865947302583056, 29.63323070313422), (16.183463971218803, 26.593296358093735), (15.862606790067051, 16.79026918696951), (26.964671198718975, 16.017360691336872)]]

    routePlanningObj = routePlanning(flyZone, noFlyZones)
    
    if args['dijkstra']:
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3), 'd')
    else:
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3))
    routePlanningObj.createMap()

"""
    Extremely good edge case to consider
    noFlyZones = [[(53.92467898230293, 50.905121078506966), (52.62260758551787, 62.8942659373792), (46.27618221864204, 50.5497070023759), (47.48715243253119, 37.136872248352006), (64.21489658733563, 44.210027470531855)], [(65.25735220121798, 13.913370847904874), (54.61252076809604, 20.968176814070276), (42.96320025094971, 25.342306457861408), (41.58939141413785, 14.388539308474764), (46.59262368088967, 6.76622804823079), (54.997070889985196, 6.4075630571338)], [(87.98294335977342, 90.32079853159945), (73.09271456833231, 97.06991841530086), (63.21400994225273, 93.86106070652957), (68.81839996496718, 81.94950909550245), (80.25719865739299, 85.13585051458573)], [(100.16793582693788, 6.909950658441927), (94.11255207263643, 14.073410561444186), (83.16509823497191, 9.183700089833671), (87.10561079823182, 0.34042690013851384), (97.70372234976503, -3.4622141761992804)], [(40.41349153246735, 73.08783428444481), (31.377286938812194, 80.08299204179757), (42.11259906295611, 66.73805329739263)], [(83.24132227357995, 57.568645858900744), (72.64614509242696, 62.87038206802974), (61.45091110437718, 58.04031902840369), (72.12025492378133, 50.98289651424176), (75.54969498942043, 40.4679761563251), (84.14361862582714, 48.46278723194654)], [(57.173369646820454, 68.86794780234784), (47.11264226072787, 72.96716947966489), (30.804096694160037, 74.51695033728022), (38.117464885810456, 59.26359176212536), (56.21037355078592, 64.45422163076718)], [(37.12212981830822, 92.75331799803605), (33.958459745116265, 100.90538106829243), (20.148996615241686, 106.94000306447127), (24.694711521547422, 92.58681986260963), (25.578421665631417, 80.0157776056027), (37.23182801526152, 81.1577145222592), (37.07809013221568, 90.20662125867214)], [(57.304392184911165, 27.338005657390937), (58.00137229578584, 39.22379057673793), (41.70029192591472, 42.34401430263042), (45.56951842198082, 26.340248872787015)], [(25.164671486032965, 28.819996909323343), (23.503676615376655, 44.43445709172494), (9.50814498192786, 41.92999015135076), (16.05669735556644, 29.69883177869935), (10.865947302583056, 29.63323070313422), (16.183463971218803, 26.593296358093735), (15.862606790067051, 16.79026918696951), (26.964671198718975, 16.017360691336872)]]
"""


