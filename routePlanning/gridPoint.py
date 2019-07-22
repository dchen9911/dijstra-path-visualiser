from settings import *

class gridPoint:
    
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.visited = False
        self.distance = LARGE_DISTANCE + 1 # some arbitrarily large distance
        self.parent = None
        self.accessible = True
    
    def __lt__(self, other):
        return self.distance <= other.distance

    def visit(self):
        self.visited= True

    def unvisit(self):
        self.visited = False
    
    def beenVisited(self):
        return self.visited
    
    def setDistance(self, dist):
        self.distance = dist
    
    def getDistance(self):
        return self.distance
    
    def setParent(self, p):
        self.parent = p

    def getParent(self):
        return self.parent

    def isAccessible(self):
        return self.accessible
    
    def setNotAccessible(self):
        self.accessible = False
    
    def setAccessible(self):
        self.accessible = True
