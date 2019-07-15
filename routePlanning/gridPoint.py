class gridPoint:
    
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.visited = False
    
    def visit(self):
        self.visited= True

    def unvisit(self):
        self.visited = False
    
    def beenVisited(self):
        return self.visited

