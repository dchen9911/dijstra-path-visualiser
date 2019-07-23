# TO DO: Check that path doesnt cut across no fly zone 
#      - This can be done quite easily by making search range just 1, OR more difficultly by dynamically changing the path
#      - Checking if line crosses a shape is hella time consuming too
# TO DO: Make 3d (somehow without requiring a shitton of time)

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
import itertools

from settings import *
from minHeap import MinHeap
from matplotlib.path import Path
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import Point, LineString
from gridPoint import gridPoint

class routePlanning:
    def __init__(self, flyZone, noFlyZone, rawWaypoints):
        self.height = HEIGHT # hard coded values for size of the grid
        self.width = WIDTH
        self.grid = [[gridPoint(j,i) for j in range(0, WIDTH)] for i in range(0, HEIGHT)]
        self.noFlyZones = []
        self.softNoFlyZones = []
        self.flyZone = Polygon(flyZone)
        self.waypoints = []
        self.rawWaypoints = rawWaypoints
        self.prevPositions = []

        for zone in noFlyZones:
            # create a polygon from the no fly zone and display it as a patch
            zonePoly = Polygon(zone)
            self.noFlyZones.append(zonePoly)

            # scaling based off size of desired boundary
            # possible to implement this based off obj.bounds
            radius = zonePoly.area ** 0.5
            # scalingFact = (radius + NO_FLY_ZONE_BOUNDARY_SIZE*2)/radius
            # softZonePoly = shapely.affinity.scale(zonePoly, xfact = scalingFact, yfact = scalingFact, zfact = scalingFact, origin = 'centroid') # create soft no fly zones by making polygons bigger
            self.softNoFlyZones.append(zonePoly)

        for point in rawWaypoints:
            self.waypoints.append(Point(point[0], point[1]))

        # preprocess points by going through each polygon and looking at each points within its bounds
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
        
        # do the same for the flyZone    
        for x in range(0, WIDTH):
            for y in range(0, HEIGHT):
                if not self.grid[y][x].isAccessible(): # slightly more efficient
                    continue
                point = Point(x,y)
                if not self.flyZone.contains(point): # if not within geofence, then also make it inaccessible
                    self.grid[y][x].setNotAccessible()
        print("Finished preprocessing")
        
    # greedyish method that tries to always move closer to endpoint if possible
    # startPoint and endPoint are tuples of floats representing coords in the form (x,y)
    def planRoute(self, startPoint, endPoint, method = 'g'):
        # pre-process all points and check how far away they are from the endpoint
        # this block can be moved to init
        self.gridDistances = 2*max(self.height, self.width)*np.ones((self.height, self.width)) # initialise them as very far away
        for x in range(0, self.width):
            for y in range(0, self.height):
                self.gridDistances[y][x] = abs(endPoint[0] - x)**2 + abs(endPoint[1] - y) ** 2
        
        self.prevPositions.append(startPoint)
        # self.dijkstraFindPath(endPoint)
        
        totalDist = 0
        # start the path finding
        totalDist = totalDist + self.dijkstraFindPath(self.rawWaypoints[0])
        self.gridReset()
        totalDist = totalDist + self.dijkstraFindPath(self.rawWaypoints[1])
        self.gridReset()
        
        # now do all permutations
        numList = [x for x in range(2, len(self.waypoints))]
        permList = list(itertools.permutations(numList))

        minDist = LARGE_DISTANCE
        minPerm = None
        startIndex = len(self.prevPositions)
        for perm in permList:
            dist = 0
            for x in perm:
                dist = dist + self.dijkstraFindPath(self.rawWaypoints[x])
                self.gridReset()
                if dist > minDist:
                    break
            if dist < minDist:
                minDist = dist
                minPerm = perm
            del self.prevPositions[startIndex:len(self.prevPositions)] # delete all the points that were added
        
        for x in minPerm:
            totalDist = totalDist + self.dijkstraFindPath(self.rawWaypoints[x])
            self.gridReset()
        totalDist = totalDist + self.dijkstraFindPath(endPoint)
        print("Total distance: " + str(totalDist))


        """
        if method == 'd':
            
        else:
            self.greedyFindPath(endPoint)
        return
        """

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

            if (currX, currY) == endPoint: # if we've reached the endpoint, then its okay to stop
                break
            # go through all the neighbours of the current point
            for x in range(currX - SEARCH_RANGE, currX + SEARCH_RANGE + 1):
                for y in range(currY - SEARCH_RANGE, currY + SEARCH_RANGE + 1):
                    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT : # check if out of bounds
                        continue
                    # check if point has been visited and whether it is accessible or not
                    currPoint = self.grid[y][x]
                    if currPoint.beenVisited() or not currPoint.isAccessible():
                        continue

                    if CHECK_LINE_INTERSECTION: # option that adds a shitton of time to the algo
                        line = LineString([[currX, currY], [x,y]])
                        continueFlag = False
                        for noFlyZone in self.softNoFlyZones:
                            if line.intersects(noFlyZone):
                                continueFlag = True
                                break
                        if continueFlag:
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

        tempPositions = []
        # now put everything into the array
        while endGridPoint != startGridPoint:
            if endGridPoint == None:
                print("no path could be found")
                return

            tempPositions.append((endGridPoint.x, endGridPoint.y))
            endGridPoint = endGridPoint.getParent()
        
        totalDistance = 0
        # reverse them, append onto list then calculate distance
        for x in range(len(tempPositions) - 1, -1, -1):
            totalDistance = totalDistance + (abs(self.prevPositions[-1][0] - tempPositions[x][0])**2 + abs(self.prevPositions[-1][1] - tempPositions[x][1])**2) ** 0.5
            self.prevPositions.append(tempPositions[x])
        return totalDistance

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
            for x in range(currPos[0] - SEARCH_RANGE, currPos[0] + SEARCH_RANGE + 1):
                for y in range(currPos[1] - SEARCH_RANGE, currPos[1] + SEARCH_RANGE + 1): 
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

    def gridReset(self):
        for x in range(0, WIDTH):
            for y in range(0,HEIGHT):
                point = self.grid[y][x]
                point.unvisit()
                point.setDistance(LARGE_DISTANCE + 1)
                point.setParent(None)

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

        i = 0
        for waypoint in self.waypoints:
            plotStyle = None 
            if i == 0: # display the first and second waypoints differently
                plotStyle = 'or'
            elif i == 1:
                plotStyle = 'ob'
            else:
                plotStyle = 'ok'
            plt.plot(waypoint.x, waypoint.y, plotStyle, markersize = 10.0)
            i = i + 1

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

def generateRandomWaypoints(n, noFlyZones):
    noFlyPolys = []
    for noFlyZone in noFlyZones:
        noFlyPolys.append(Polygon(noFlyZone))
    
    waypoints = []

    cnt = 0
    while len(waypoints) < n:
        if cnt > 20*n:
            print("Error creating waypoints") # i.e. waypoints keep getting generated inside the polygon
            return None
        cnt = cnt + 1

        x = round(random.randrange(5, WIDTH - 5))
        y = round(random.randrange(5, HEIGHT - 5))
        contFlag = False
        for noFlyPoly in noFlyPolys:
            if noFlyPoly.contains(Point(x,y)):
                contFlag = True
                break
        if not contFlag:
            waypoints.append((x,y))
    
    return waypoints
        



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plans the route')
    parser.add_argument('-d', '--dijkstra', required=False, help='Use dijkstra rather than greedy', action='store_true')
    parser.add_argument('-m', '--manual', required=False, help='Manually set waypoints', action='store_true')
    args = vars(parser.parse_args())

    flyZone = [(1,1),(WIDTH,5), (WIDTH,HEIGHT), (5,HEIGHT)]
    if args['manual']:
        waypoints = [(28, 71), (60, 40), (91, 60), (61, 27), (49, 93)]
        noFlyZones = [[(31.55071563114219, 63.99392635003229), (28.0043046569918, 70.52088094544614), (21.708559527514847, 74.83133878158526), (9.670919213879543, 72.13468557138614), (11.939731152286992, 60.343676048061134), (18.571067357795748, 56.4664488935459), (23.88437110326891, 56.95996932636844)], [(94.21362564023171, 72.97201743802569), (83.75674718882733, 79.78328211975483), (91.14219752397527, 74.28613484130784), (89.36332127053036, 62.40708751287595)], [(63.72401647769463, 48.75348509074943), (57.057797560042744, 66.52076307322105), (47.19355807584869, 57.87122190091681), (47.60517649157064, 49.98294886661027), (55.381510901631664, 46.95192493443756)], [(24.067421161892952, 48.33682682226254), (13.940596727891737, 49.62558401498617), (12.320642997910937, 41.76572497280503), (21.65270719237404, 35.97795253536659), (29.43613599933095, 46.936727585626855)], [(50.754102187370044, 65.28779733508749), (36.27611099824477, 67.97169198826845), (29.92645350427571, 68.04904500242093), (23.31278349530581, 63.174515941111466), (24.9318024587695, 57.80016722671616), (21.41843856842764, 40.83683044293433), (37.38635574302759, 45.20712831598443)], [(14.98439553136816, 56.10606896182914), (1.976845472291398, 49.41188536729869), (-7.627296123899317, 38.05259766624492), (7.361910878243659, 31.977226231955015)], [(26.767211839372173, 79.2443623077232), (7.312190293481585, 95.12418019335482), (9.458972072304125, 72.44363079736587), (15.631151035335657, 82.74789274759077)], [(40.461235611016235, 64.94918131065938), (40.194851870884946, 72.20991702503359), (33.64213346754096, 80.09863101678762), (32.4351396481954, 71.52530598514622), (28.984302988467327, 67.36953486233321), (29.632622038590164, 59.71986074500658), (38.432689292653066, 56.65292674085477)], [(80.55277731166653, 61.31859566741921), (74.42286288512058, 65.58023809398546), (67.84267525115061, 64.07251617480895), (59.19710754879486, 51.971545218198905), (68.46946422636893, 47.73260579344434), (78.40691598542303, 52.12361032336971)], [(23.87268323653852, 90.24929816686665), (20.022012873719305, 107.48591774895256), (13.797069922662155, 94.58718277816958), (11.90574198164241, 90.12053179258058), (17.340816554598327, 88.39219130347702)]]
    else:
        while True:
            noFlyZones = generateRandomNoFlyZones(POLYGON_NUMBER,POLYGON_SIZE)
            waypoints = generateRandomWaypoints(5, noFlyZones) # keep trying to generate valid waypoints
            if waypoints != None:
                break
    print(waypoints)
    print()
    print(noFlyZones)

    routePlanningObj = routePlanning(flyZone, noFlyZones, waypoints)
    if args['dijkstra']:
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3), 'd')
    else:
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3))
    
    routePlanningObj.createMap()

"""
    Extremely good edge case to consider
    noFlyZones = [[(53.92467898230293, 50.905121078506966), (52.62260758551787, 62.8942659373792), (46.27618221864204, 50.5497070023759), (47.48715243253119, 37.136872248352006), (64.21489658733563, 44.210027470531855)], [(65.25735220121798, 13.913370847904874), (54.61252076809604, 20.968176814070276), (42.96320025094971, 25.342306457861408), (41.58939141413785, 14.388539308474764), (46.59262368088967, 6.76622804823079), (54.997070889985196, 6.4075630571338)], [(87.98294335977342, 90.32079853159945), (73.09271456833231, 97.06991841530086), (63.21400994225273, 93.86106070652957), (68.81839996496718, 81.94950909550245), (80.25719865739299, 85.13585051458573)], [(100.16793582693788, 6.909950658441927), (94.11255207263643, 14.073410561444186), (83.16509823497191, 9.183700089833671), (87.10561079823182, 0.34042690013851384), (97.70372234976503, -3.4622141761992804)], [(40.41349153246735, 73.08783428444481), (31.377286938812194, 80.08299204179757), (42.11259906295611, 66.73805329739263)], [(83.24132227357995, 57.568645858900744), (72.64614509242696, 62.87038206802974), (61.45091110437718, 58.04031902840369), (72.12025492378133, 50.98289651424176), (75.54969498942043, 40.4679761563251), (84.14361862582714, 48.46278723194654)], [(57.173369646820454, 68.86794780234784), (47.11264226072787, 72.96716947966489), (30.804096694160037, 74.51695033728022), (38.117464885810456, 59.26359176212536), (56.21037355078592, 64.45422163076718)], [(37.12212981830822, 92.75331799803605), (33.958459745116265, 100.90538106829243), (20.148996615241686, 106.94000306447127), (24.694711521547422, 92.58681986260963), (25.578421665631417, 80.0157776056027), (37.23182801526152, 81.1577145222592), (37.07809013221568, 90.20662125867214)], [(57.304392184911165, 27.338005657390937), (58.00137229578584, 39.22379057673793), (41.70029192591472, 42.34401430263042), (45.56951842198082, 26.340248872787015)], [(25.164671486032965, 28.819996909323343), (23.503676615376655, 44.43445709172494), (9.50814498192786, 41.92999015135076), (16.05669735556644, 29.69883177869935), (10.865947302583056, 29.63323070313422), (16.183463971218803, 26.593296358093735), (15.862606790067051, 16.79026918696951), (26.964671198718975, 16.017360691336872)]]
"""


