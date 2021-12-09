import numpy as np

class DStar:
    def __init__(self, start, goal, map):
        self.start = start 
        self.goal = goal 
        self.map = map
        
        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0

        for i in range(0, self.map.shape[0]):
            for j in range(0, self.map.shape[1]):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.goal] = 0.0
        self.U[self.goal] = self.CalculateKey(self.goal)
        self.path = None
    
    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.GetH(self.start, s) + self.km, min(self.g[s], self.rhs[s])]

    def GetH(self, start, s):
        return abs(start[0] - s[0]) + abs(start[1] - s[1])

    def TopKey(self):
        s = min(self.U, key=self.U.get)
        return s, self.U[s]

    def GetCost(self, start, goal):
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def ComputeShortestPath(self):
        while True:
            s, valueOfS = self.TopKey()

            if valueOfS > self.CalculateKey(self.start) and self.rhs[self.start] == self.g[self.start]:
                break

            self.U.pop(s)

            if  valueOfS < self.CalculateKey(s):
                self.U[s] = self.CalculateKey(s)
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.GetNeighbours(s):
                    self.UpdateVertex(x)
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for x in self.GetNeighbours(s):
                    self.UpdateVertex(x)

    def GetNeighbours(self, s):
        nei_list = set()
        motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                   (1, 0), (1, -1), (0, -1), (-1, -1)]
        for u in motions:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if  (s_next[0] >= 0 and s_next[0] < self.map.shape[0] \
                and s_next[1] >= 0 and s_next[1] < self.map.shape[1]) \
                and self.map[s_next[0]][s_next[1]] != 1:
                nei_list.add(s_next)

        return nei_list

    def UpdateVertex(self, s):
        if s != self.goal:
            self.rhs[s] = float("inf")

            for x in self.GetNeighbours(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.GetCost(s, x))
           

        
        if s in self.U:
            self.U.pop(s)

        if self.g[s] != self.rhs[s]:
            self.U[s] = self.CalculateKey(s)

    def NewObst(self, last, curr, x, y):

        self.km += self.GetH(last, curr)
        
        if self.map[(x,y)] == 0:
            self.map[(x,y)] = 1
            self.g[(x, y)] = float("inf")
            self.rhs[(x, y)] = float("inf")
        else:
            self.map[(x,y)] = 0
            self.UpdateVertex((x, y))
        for s in self.GetNeighbours((x, y)):
            self.UpdateVertex(s)


    def Run(self):
        self.ComputeShortestPath()
        print(self.ExtractPath())
    
    def Collision(self, start, goal):
        if self.map[start[0]][start[1]] == 1:
            return True
        if self.map[goal[0]][goal[1]] == 1:
            return True

        if start[0] != goal[0] and start[1] != goal[1]:
            if goal[0] - start[0] == start[1] - goal[1]:
                s1 = (min(start[0], goal[0]), min(start[1], goal[1]))
                s2 = (max(start[0], goal[0]), max(start[1], goal[1]))
            else:
                s1 = (min(start[0], goal[0]), max(start[1], goal[1]))
                s2 = (max(start[0], goal[0]), min(start[1], goal[1]))

            if self.map[s1[0]][s1[1]] == 1 or self.map[s2[0]][s2[1]] == 1:
                return True

        
        return False

    def GetStart(self, current):
      
        g_list = {}
        for x in self.GetNeighbours(current):
            if not self.Collision(current, x):
                g_list[x] = self.g[x]
        start = min(g_list, key=g_list.get)

        return start



    def ExtractPath(self):
        path = [self.start]
        s = self.start

        while True:
            g_list = {}
            for x in self.GetNeighbours(s):
                if not self.Collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.goal:
                break

        self.path = path
        return path