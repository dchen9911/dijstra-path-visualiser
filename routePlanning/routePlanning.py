# TO DO: incorporate a list of waypoints that have to be followed
# TO DO: linearly interpolate the path so less way points are given to the drone
# TO DO: Make dijkstra search in a 5x5 box around point for better 
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

from minHeap import MinHeap
from matplotlib.path import Path
from descartes import PolygonPatch
from shapely.geometry.polygon import LinearRing, Polygon
from shapely.geometry import Point

from gridPoint import gridPoint

NO_FLY_ZONE_BOUNDARY_SIZE = 2
WIDTH = 100
HEIGHT = 100
POLYGON_NUMBER = 15
POLYGON_SIZE = 9

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
        count = 0
        for x in range(0, WIDTH):
            for y in range (0, HEIGHT):
                self.grid[y][x].setDistance(self.grid[y][x].getDistance() + count)
                pq.insert(self.grid[y][x], self.grid[y][x].getDistance())
                count = count + 1
        
        while pq.counter != 0:
            currGridPoint = pq.del_min()
            if currGridPoint.getDistance() > 1000000: # these are the points that cannot be reached from the start point
                break
            currGridPoint.visit()
            currX = currGridPoint.x
            currY = currGridPoint.y

            # go through all the neighbours of the current point
            for x in range(currX - 1, currX + 2):
                for y in range(currY - 1, currY + 2): 
                    currPos = (x,y)
                    if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT or self.grid[y][x].beenVisited(): # check if we've already visited or out of bounds
                        continue
                    continueFlag = False
                    for noFlyZone in self.softNoFlyZones: # check to see if point is in a no fly zone or not within the yes fly zone
                        if noFlyZone.contains(Point(currPos)):
                            continueFlag = True
                            break
                    if continueFlag:
                        continue
                    if not self.flyZone.contains(Point(currPos)):
                        continue
                    
                    currPoint = self.grid[y][x]
                    oldDist = currPoint.getDistance()
                    newDist = currGridPoint.getDistance() + (abs(x - currX)**2 + abs(y - currY)**2) ** 0.5
                    if newDist < oldDist:
                        currPoint.setDistance(newDist)
                        pq.insert(currPoint, currPoint.getDistance())
                        currPoint.setParent(currGridPoint)
        
        endGridPoint = self.grid[endPoint[1]][endPoint[0]]
        if endGridPoint.getDistance() > 1000000:
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
            
        # bulk of the path finding
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
    noFlyZones = generateRandomNoFlyZones(POLYGON_NUMBER,POLYGON_SIZE)
    print(noFlyZones)
    
    routePlanningObj = routePlanning(flyZone, noFlyZones)
    
    if args['dijkstra']:
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3), 'd')
    else:
        routePlanningObj.planRoute((2,2), (WIDTH - 3, HEIGHT - 3))
    routePlanningObj.createMap()

"""
    noFlyZones = [[(32.76760500804986, 70.0209447504084), (25.09857190136023, 70.35169643515728), (25.633972949409838, 64.02954970830679)], [(76.70901348460869, 80.7172212972922), (74.2498524330669, 87.57687919565643), (68.07951516028578, 81.79841208262137), (75.1509579124204, 76.85843762819239)], [(90.42780350790925, 57.9356399802175), (88.11299074654477, 63.18770183628909), (82.13180268211882, 62.96972015261172), (79.98077349792366, 55.58826569962246), (88.22597303154953, 55.852642181400256)], [(59.14031260580107, 44.26734595607289), (52.406314622921855, 51.483493123463894), (50.97922659561319, 44.672402236395904)], [(49.899621903442146, 45.25431039146686), (40.51476093497907, 51.608745469896846), (41.59156697900454, 44.68934798719302)], [(64.54029983130572, 59.56258393850445), (57.95413564087323, 59.68159580000331), (58.75294535062098, 56.41638199239141), (60.84577843030448, 49.98432977837875)], [(67.73850160659585, 65.93095562411187), (66.11465974577946, 68.7792762563866), (64.48720721376512, 64.53263651322936), (59.466733642718076, 63.652616552957554), (62.77107268543308, 61.747653442016144), (66.63149026519632, 61.10239484247401)], [(96.51895249760034, 58.03894738561338), (91.50929620128193, 59.72897718168009), (86.43475315880207, 56.52080963423007), (91.38719422137169, 52.85130826702888)], [(99.52486191086899, 74.3682842147012), (96.96169737705002, 81.24525273042832), (88.27748860514755, 75.07544076475008)],
[(91.711842072336, 93.17756473801893), (82.6112207560941, 99.54993171325725), (84.09260971329402, 89.58601237075774)], [(56.74181966495754, 33.6704804838274), (48.17934766384963, 36.33683932418062), (51.94660927744403, 26.477845669353076)], [(71.46114218367009, 90.13915193986907), (60.893868646315305, 84.59146699268857), (71.69646807358936, 83.38958025055443)], [(63.738175642292354, 98.1874151878994), (62.55377446512684, 102.3236979226125), (58.58831433550984, 105.20394964977928), (53.77486721669263, 99.26117640296522), (54.56053711210503, 95.86041191462905), (57.608785314939745, 93.81514056006554), (62.2216295469646, 90.8506444461608)], [(51.37840829559036, 36.572376451182194), (47.06022791115201, 42.65927981233223), (42.70121770087547, 36.25378390939924), (42.97621803521859, 32.5347578849977), (49.66619211890273, 30.31341691280898)], [(101.29735844959673, 59.6285250202436), (94.1010833429035, 65.56897857461682), (91.90668704719349,
60.539639329201755), (93.16955740709525, 58.949033706113774), (97.02107964404848, 55.72033937683914)], [(87.7531045359064, 26.52803907210708), (86.00520997845803, 29.611218460745857), (80.61162131651392, 27.32370195310205), (81.02936359143114, 21.320481724348674), (84.81717191036495, 20.986723297913638)], [(50.36193363550362, 74.38471947051937), (49.44368555239464, 76.70646693837863), (48.626493324122876, 80.50971249271336), (45.78518747418626, 79.874129380561), (40.058474899443574, 73.9255327007024), (42.77494446285655, 67.08002183371367), (51.64470673489732, 67.69104803592965)], [(35.12095907101471, 27.15749389400972), (31.75129213825791, 30.19546699524588), (26.215204341773568, 29.302416006305723),
(26.93452205610423, 24.901745986763792), (30.315303322659318, 22.02936304582731), (35.27374398560093, 24.26036366280309)], [(32.528754250420356, 57.69780243670025), (28.313303397591152, 63.28626566916881), (24.102621312204185, 55.811712485324186), (31.36007759555655, 52.151105121740166), (35.08165994775537, 59.15012912997044)], [(46.35828309306213, 93.82841811441833), (40.31772362336212, 97.71332853579531), (42.325188850049045, 92.02551391794735), (46.19383448093302, 85.95255287587825)], [(70.98888870110534, 55.04464006889236), (70.75937018370402, 60.75416275184507), (67.26015698606753, 62.183278611218675), (62.48484521406049, 58.389847083444494)], [(98.23879285623912, 60.5253360275126), (92.74967257228046, 63.02381159882326), (88.6495775114501, 60.92296591340397), (89.29383198503096, 54.13102898882416), (96.33872527928702, 55.60764443439465)], [(74.08492632660965, 30.020607431550328), (73.09853785932216, 32.64130066638296), (69.50406841936571, 33.44000428633526), (64.4592709216783, 30.640108827635107), (70.357613979795, 26.655419171674286)], [(38.99179035257599, 65.15884784260665), (38.81337851179376, 68.8658824919724), (31.458049313429587, 71.68436934070661), (28.963986063713953, 66.47629363777338), (33.92574901360399, 61.92718189363565)], [(68.496762729322, 78.77127241304305), (66.06239758158114, 83.50880600503257), (62.566149465850366, 84.15750526892919), (57.10094532772577, 84.42821440279312),
(58.11387243001579, 78.41156950793977), (62.858697957893014, 73.57769569582669)], [(86.51054183631422, 51.84451454948209), (74.90077191174923, 56.911282685480074), (75.77729276359969, 49.67227158702777)], [(37.79009988995943, 29.883315739505683), (37.743139203733456, 36.84293914714557), (32.955941218786386, 36.673669531053015)], [(100.86590221817602, 55.037231314984616), (94.97744155982615, 58.12654939527161), (92.96009515517073, 45.50655400780197), (105.31653942393059, 47.20966349195742)], [(51.81528497726535, 23.706649884839216), (47.812212657199304, 27.266648262012986), (43.829692995866566, 28.44732702670832), (39.08707204587909, 24.226970962842188), (40.95743355278216, 20.165168414444334), (46.39753069133169, 18.737276641201394), (48.88185095873331, 21.4456354765303)], [(44.14313407834957, 49.356762815690516), (45.70826700798642, 55.1219127430466), (40.83996201704601, 57.91347166025867), (33.51539216367227, 53.56703889288411), (40.400915770418294, 49.82914787441269)]]
"""


