
NO_FLY_ZONE_BOUNDARY_SIZE = 2 # pixel width of the buffer zone
WIDTH = 100 # pixel size of simulation
HEIGHT = 100
POLYGON_NUMBER = 10
POLYGON_SIZE = 9
SEARCH_RANGE = 3 # number of pixels that can be jumped to (increases smoothness at risk of jumping across nofly zones)

LARGE_DISTANCE = 100000000 # arbitrary constant, used in dijkstras as "infinite" distance

CHECK_LINE_INTERSECTION = False # check if lines on the path intersect (increases computation time by a shitton)