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
        self.finalPath = []
        self.gridScanDistances = []

        # initialise the no fly zones and waypoints
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
        
    # startPoint and endPoint are tuples of floats representing coords in the form (x,y)
    def planRoute(self, startPoint, endPoint):
        self.finalPath.append(startPoint)
        totalDist = 0

        # start the path finding by finding distance to first two paths
        totalDist = totalDist + self.dijkstraFindPath(startPoint, self.rawWaypoints[0], self.finalPath)
        totalDist = totalDist + self.dijkstraFindPath(self.rawWaypoints[0], self.rawWaypoints[1], self.finalPath)
        
        waypointDistances = {}
        allPaths = {}
        timebig = time.time_ns()
        # preprocess to find distance between any two points
        for i in range(1, len(self.rawWaypoints)):
            for j in range(i + 1, len(self.rawWaypoints)):
                pointsInPath = [self.rawWaypoints[i]] # store the beginning point too
                dist = self.dijkstraFindPath(self.rawWaypoints[i], self.rawWaypoints[j], pointsInPath)
                waypointDistances[(i,j)] = dist # store the distance in a dictionary
                waypointDistances[(j,i)] = dist
                allPaths[(i,j)] = pointsInPath  # also store the path taken
    
        print("Preprocessing all paths time: " + str((time.time_ns() - timebig)/1000000000))
        
        # now do all permutations and check the distance required for every perm
        numList = [x for x in range(2, len(self.waypoints))]
        permList = list(itertools.permutations(numList))

        minDist = LARGE_DISTANCE
        minPerm = None
        
        # go through each permutation
        for perm in permList:
            dist = 0
            prevWaypoint = 1 # always start out at the first waypoint
            for x in perm:
                dist = dist + waypointDistances[(prevWaypoint,x)]
                if dist > minDist:
                    break
                prevWaypoint = x
                i = i + 1

            if dist < minDist:
                minDist = dist
                minPerm = perm
        self.waypointOrder = [0, 1] # stores what order the waypoints will be traversed in
        self.waypointOrder = self.waypointOrder + list(minPerm)

        prevWaypoint = 1
        for currWaypoint in minPerm:
            if currWaypoint < prevWaypoint:
                allPaths[(currWaypoint, prevWaypoint)].reverse()
                del allPaths[(currWaypoint, prevWaypoint)][0]
                self.finalPath = self.finalPath + allPaths[(currWaypoint, prevWaypoint)]
            elif prevWaypoint < currWaypoint:
                del allPaths[(prevWaypoint, currWaypoint)][0]
                self.finalPath = self.finalPath + allPaths[(prevWaypoint, currWaypoint)]
            else:
                print("current and previous waypoints are the same")
                raise Exception()
            totalDist = totalDist + waypointDistances[(prevWaypoint, currWaypoint)]
            prevWaypoint = currWaypoint
        
        totalDist = totalDist + self.dijkstraFindPath(self.finalPath[-1], endPoint, self.finalPath)

        
        timebig = time.time_ns()
        self.optimiseFinalPath()
        print("Optimisation time: " + str((time.time_ns() - timebig)/1000000000))
        
        print("Unoptimised distance: " + str(totalDist))
        totalDist = 0
        for i in range(1, len(self.finalPath)):
            totalDist = totalDist + (abs(self.finalPath[i][0] - self.finalPath[i-1][0])**2 + abs(self.finalPath[i][1] - self.finalPath[i-1][1])**2) ** 0.5
        print("Optimised distance: " + str(totalDist))


    def dijkstraFindPath(self, startPoint, endPoint, pointsInPath, checkLineIntersection = False, searchRange = DEFAULT_SEARCH_RANGE):
        # pre-calculate distances in a search range grid
        self.gridScanDistances = []
        for i in range(-searchRange, searchRange + 1):
            tmp = []
            for j in range(-searchRange, searchRange + 1):
                tmp.append((i ** 2 + j**2)**0.5)
            self.gridScanDistances.append(tmp)

        self.grid[startPoint[1]][startPoint[0]].setDistance(0)
        pq = MinHeap()
        # create the priority queue
        for x in range(0, WIDTH):
            for y in range (0, HEIGHT):
                pq.insert(self.grid[y][x], self.grid[y][x].getDistance())
        
        count = 0
        while pq.counter != 0:
            currGridPoint = pq.del_min()
            if currGridPoint.getDistance() > LARGE_DISTANCE: # these are the points that cannot be reached from the start point
                print("points covered: " + str(count))
                print("Destination point cannot be reached")
                raise Exception()
                break
            currGridPoint.visit()
            currX = currGridPoint.x
            currY = currGridPoint.y

            if (currX, currY) == endPoint: # if we've reached the endpoint, then its okay to stop
                break
            
            # go through all the neighbours of the current point
            for i in range(-searchRange, searchRange + 1):
                y = i + currY
                for j in range(- searchRange,  searchRange + 1):
                    x = j + currX

                    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT : # check if out of bounds
                        continue

                    # check if point has been visited and whether it is accessible or not
                    currPoint = self.grid[y][x]
                    if currPoint.beenVisited() or not currPoint.isAccessible():
                        continue
                    if checkLineIntersection: # option that adds a shitton of time to the algo
                        line = LineString([[currX, currY], [x,y]])
                        continueFlag = False
                        for noFlyZone in self.softNoFlyZones:
                            if line.intersects(noFlyZone):
                                continueFlag = True
                                break
                        if continueFlag:
                            continue

                    oldDist = currPoint.getDistance()
                    newDist = currGridPoint.getDistance() + self.gridScanDistances[i + searchRange][j + searchRange]
                    if newDist < oldDist:
                        currPoint.setDistance(newDist)
                        pq.insert(currPoint, currPoint.getDistance())
                        currPoint.setParent(currGridPoint)
            count = count + 1

        # check if the endpoint has actually been visited (probs not necessary but why not)
        endGridPoint = self.grid[endPoint[1]][endPoint[0]]
        if endGridPoint.getDistance() > LARGE_DISTANCE:
            print("no path could be found")
            raise Exception()
            return
        
        # now find the shortest path by going up the tree
        startGridPoint = self.grid[startPoint[1]][startPoint[0]] 
        tempPath = []
        while endGridPoint != startGridPoint:
            if endGridPoint == None:
                print("no path could be found between following two points: ", end = "")
                print(startPoint, end = "")
                print(endPoint)
                raise Exception()
                return
            tempPath.append((endGridPoint.x, endGridPoint.y))
            endGridPoint = endGridPoint.getParent()
        tempPath.append((startGridPoint.x, startGridPoint.y))

        totalDistance = 0
        # reverse them, append onto list then calculate distance
        for x in range(len(tempPath) - 2, -1, -1):
            totalDistance = totalDistance + (abs(tempPath[x+1][0] - tempPath[x][0])**2 + abs(tempPath[x+1][1] - tempPath[x][1])**2) ** 0.5
            pointsInPath.append(tempPath[x])

        self.gridReset()
        return totalDistance

    # make sure final path doesnt cross any no flyzones and also make the path smoother
    def optimiseFinalPath(self):
        x = 1   # index of current point
        waypointIndex = 0

        while True:
            if x + 1 >= len(self.finalPath):
                print("reached the end")
                break 

            stepSize = None
            saveFlag = False
            for i in range(0, DEFAULT_STEP_SIZE):
                # if its a waypoint, then break
                stepSize = i
                if x + i >= len(self.finalPath) - 1 or self.finalPath[x + i] == self.rawWaypoints[self.waypointOrder[waypointIndex]]:                        
                    saveFlag = True
                    if not waypointIndex >= len(self.rawWaypoints) - 1: # dont go over too much
                        waypointIndex = waypointIndex + 1
                    break

            # make a line from the previous point to the next one ahead then check if it intersects something
            line = LineString([[self.finalPath[x - 1][0], self.finalPath[x - 1][1]], [self.finalPath[x + stepSize][0],self.finalPath[x + stepSize][1]]])
            intersectFlag = False
            for noFlyZone in self.softNoFlyZones:
                if line.intersects(noFlyZone):
                    intersectFlag = True
                    break
            
            for i in range(stepSize - 1, 0, -1):
                del self.finalPath[x + i]

            # if the line does intersect, then try connecting current point with next point
            if intersectFlag:
                line = LineString([[self.finalPath[x][0], self.finalPath[x][1]], [self.finalPath[x + 1][0],self.finalPath[x + 1][1]]])
            else:
                del self.finalPath[x]
                if saveFlag:
                    x = x + 1
                continue

            # check if the new line intersects
            intersectFlag = False
            for noFlyZone in self.softNoFlyZones:
                if line.intersects(noFlyZone):
                    intersectFlag = True
                    break
            
            # if this line does intersect AGAIN, then use dijkstra to find a non-intersecting path and attach that on
            if intersectFlag:
                tmpPath = []
                self.dijkstraFindPath(self.finalPath[x], self.finalPath[x + 1], tmpPath, True, stepSize)
                del tmpPath[-1]
                self.finalPath = self.finalPath[:x + 1] + tmpPath + self.finalPath[x + 1:]
                x = x + len(tmpPath) + 1
            else:
                x = x + 1
            if saveFlag:
                x = x + 1

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
        xvals = [row[0] for row in self.finalPath]
        yvals = [row[1] for row in self.finalPath]

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
    parser.add_argument('-m', '--manual', required=False, help='Manually set waypoints', action='store_true')
    args = vars(parser.parse_args())

    flyZone = [(0,0),(WIDTH,5), (WIDTH,HEIGHT), (5,HEIGHT)]
    if args['manual']:
        #waypoints = waypoints = [(4,2), (5,7), (8,8), (10,9), (12,12)]
        #noFlyZones = []
        waypoints = [(28, 71), (60, 40), (91, 60), (61, 27), (49, 93)]
        noFlyZones = [[(31.55071563114219, 63.99392635003229), (28.0043046569918, 70.52088094544614), (21.708559527514847, 74.83133878158526), (9.670919213879543, 72.13468557138614), (11.939731152286992, 60.343676048061134), (18.571067357795748, 56.4664488935459), (23.88437110326891, 56.95996932636844)], [(94.21362564023171, 72.97201743802569), (83.75674718882733, 79.78328211975483), (91.14219752397527, 74.28613484130784), (89.36332127053036, 62.40708751287595)], [(63.72401647769463, 48.75348509074943), (57.057797560042744, 66.52076307322105), (47.19355807584869, 57.87122190091681), (47.60517649157064, 49.98294886661027), (55.381510901631664, 46.95192493443756)], [(24.067421161892952, 48.33682682226254), (13.940596727891737, 49.62558401498617), (12.320642997910937, 41.76572497280503), (21.65270719237404, 35.97795253536659), (29.43613599933095, 46.936727585626855)], [(50.754102187370044, 65.28779733508749), (36.27611099824477, 67.97169198826845), (29.92645350427571, 68.04904500242093), (23.31278349530581, 63.174515941111466), (24.9318024587695, 57.80016722671616), (21.41843856842764, 40.83683044293433), (37.38635574302759, 45.20712831598443)], [(14.98439553136816, 56.10606896182914), (1.976845472291398, 49.41188536729869), (-7.627296123899317, 38.05259766624492), (7.361910878243659, 31.977226231955015)], [(26.767211839372173, 79.2443623077232), (7.312190293481585, 95.12418019335482), (9.458972072304125, 72.44363079736587), (15.631151035335657, 82.74789274759077)], [(40.461235611016235, 64.94918131065938), (40.194851870884946, 72.20991702503359), (33.64213346754096, 80.09863101678762), (32.4351396481954, 71.52530598514622), (28.984302988467327, 67.36953486233321), (29.632622038590164, 59.71986074500658), (38.432689292653066, 56.65292674085477)], [(80.55277731166653, 61.31859566741921), (74.42286288512058, 65.58023809398546), (67.84267525115061, 64.07251617480895), (59.19710754879486, 51.971545218198905), (68.46946422636893, 47.73260579344434), (78.40691598542303, 52.12361032336971)], [(23.87268323653852, 90.24929816686665), (20.022012873719305, 107.48591774895256), (13.797069922662155, 94.58718277816958), (11.90574198164241, 90.12053179258058), (17.340816554598327, 88.39219130347702)]]
    else:
        while True:
            noFlyZones = generateRandomNoFlyZones(POLYGON_NUMBER,POLYGON_SIZE)
            waypoints = generateRandomWaypoints(NUMBER_OF_WAYPOINTS, noFlyZones) # keep trying to generate valid waypoints
            if waypoints != None:
                break
    print(waypoints)
    print()
    print(noFlyZones)
    time1 = time.time_ns()
    routePlanningObj = routePlanning(flyZone, noFlyZones, waypoints)
    preProcessingTime = (time.time_ns() - time1)/1000000000
    print("Preprocessing time: " + str(preProcessingTime))
    if args['manual']:
        #routePlanningObj.planRoute((1,1), (13, 13), 'd')
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3))
    else:
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3))
    planTime = (time.time_ns() - time1)/1000000000 - preProcessingTime
    print("Total route planning time in seconds: "+ str(planTime))
    routePlanningObj.createMap()

"""
    Extremely good edge case to consider
    noFlyZones = [[(64.35877176192193, 82.45924642930628), (53.76037785714636, 103.69192029914122), (57.799999960129824, 81.00896430470684)], [(34.696505057831864, 90.5705291381587), (37.2881465919467, 92.77717928805147), (31.53420829735762, 97.73093472284124), (29.54308976911873, 96.83814913789026), (23.06555672364768, 95.05120873313732), (29.20814936288664, 89.98546311229313), (28.497104545185472, 81.16853315028357)], [(101.99106468256493, 19.123695789838855), (96.81324224464176, 22.67664610428111), (92.63106634645374, 18.81458246410471), (86.80035937219176, 24.126988931940293), (81.20731915143767, 16.401130012129315), (85.82403678400779, 7.0151587968875795), (93.15941694106131, 13.00353598795788)], [(56.119682814611785, 94.12363546092249), (50.181202010934555, 106.49041593794382), (48.52102117871587, 89.32643869447885)], [(46.211944555288895, 53.33212967194265), (36.783988600089316, 62.72765104070729), (30.813103090091595, 62.794543560530165), (26.52838238405523, 59.85768693564278), (21.654507591322627, 57.444760003659596), (24.949599129969833, 51.25821247637504), (32.95980515402602, 40.84201218729662)], [(28.896097311987624, 16.36963573789569), (25.041577440702405, 19.790832407643883), (21.975812189691773, 23.99925486739933), (10.004005861363495, 20.929211209343315), (8.642995515662754, 15.06485603456789), (14.47991428139322, 6.959979492513325), (23.749038941555998, 7.7719856525971585)], [(61.23254897028023, 9.089993088638147), (71.39058682635292, 14.266866152134318), (66.33937354100442, 22.19389883184396), (54.526865648558434, 23.382487962187355), (56.26247002548984, 9.720011119736343), (55.02125294931314, -0.37721558268333055), (61.12572326791406, 3.020258510683947)], [(96.24813443795695, 70.22943646460412), (88.12545476318911, 84.19353160446984), (82.25805775462439, 73.54527824246925), (87.13448884238521, 66.00676252551206), (96.79779835745603, 59.07729457876697)], [(76.21745828018157, 69.4113175669618), (73.4296494645441, 85.52062008578466), (66.76750427437754, 80.37913596399409), (51.898326117053344, 74.1731022791034), (67.99739839731885, 68.27246005544464)], [(94.90006566905254, 26.008295346567987), (84.44698738688959, 34.70492466985988), (80.11586439160969, 37.13019569868066), (74.88441314008861, 28.718765629549914), (70.57851332215073, 20.770818070519212), (79.88598271116159, 15.739377334255591), (89.0560753600714, 23.13531371013926)]]
    waypoints = [(56, 60), (91, 43), (10, 41), (53, 44), (27, 79)]
    noFlyZones = [[(53.92467898230293, 50.905121078506966), (52.62260758551787, 62.8942659373792), (46.27618221864204, 50.5497070023759), (47.48715243253119, 37.136872248352006), (64.21489658733563, 44.210027470531855)], [(65.25735220121798, 13.913370847904874), (54.61252076809604, 20.968176814070276), (42.96320025094971, 25.342306457861408), (41.58939141413785, 14.388539308474764), (46.59262368088967, 6.76622804823079), (54.997070889985196, 6.4075630571338)], [(87.98294335977342, 90.32079853159945), (73.09271456833231, 97.06991841530086), (63.21400994225273, 93.86106070652957), (68.81839996496718, 81.94950909550245), (80.25719865739299, 85.13585051458573)], [(100.16793582693788, 6.909950658441927), (94.11255207263643, 14.073410561444186), (83.16509823497191, 9.183700089833671), (87.10561079823182, 0.34042690013851384), (97.70372234976503, -3.4622141761992804)], [(40.41349153246735, 73.08783428444481), (31.377286938812194, 80.08299204179757), (42.11259906295611, 66.73805329739263)], [(83.24132227357995, 57.568645858900744), (72.64614509242696, 62.87038206802974), (61.45091110437718, 58.04031902840369), (72.12025492378133, 50.98289651424176), (75.54969498942043, 40.4679761563251), (84.14361862582714, 48.46278723194654)], [(57.173369646820454, 68.86794780234784), (47.11264226072787, 72.96716947966489), (30.804096694160037, 74.51695033728022), (38.117464885810456, 59.26359176212536), (56.21037355078592, 64.45422163076718)], [(37.12212981830822, 92.75331799803605), (33.958459745116265, 100.90538106829243), (20.148996615241686, 106.94000306447127), (24.694711521547422, 92.58681986260963), (25.578421665631417, 80.0157776056027), (37.23182801526152, 81.1577145222592), (37.07809013221568, 90.20662125867214)], [(57.304392184911165, 27.338005657390937), (58.00137229578584, 39.22379057673793), (41.70029192591472, 42.34401430263042), (45.56951842198082, 26.340248872787015)], [(25.164671486032965, 28.819996909323343), (23.503676615376655, 44.43445709172494), (9.50814498192786, 41.92999015135076), (16.05669735556644, 29.69883177869935), (10.865947302583056, 29.63323070313422), (16.183463971218803, 26.593296358093735), (15.862606790067051, 16.79026918696951), (26.964671198718975, 16.017360691336872)]]

"""


