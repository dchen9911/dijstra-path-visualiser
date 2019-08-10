from shapely.geometry.polygon import Polygon
from settings import *

class noFlyZone:
    def __init__(self, coords, bottom, top):
        self.coords = coords # these coords don't wrap
        self.bottom = bottom
        self.top = top
        self.polygonObj = Polygon(coords)
        self.inGrid2Ds = range(bottom, top + 1) # all the planes (2d grids) it is in (by z index), might change later