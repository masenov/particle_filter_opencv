import numpy as np
import cv2
import math
import random


def int_(a):
        return int(round(a))

# Maze is 20x16
class Maze(object):
    def __init__(self, scaling=40):
        self.maze=((2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 2, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2, 2),
                (2, 0, 0, 0, 0, 0, 0, 2, 0, 2, 1, 1, 1, 1, 1, 2),
                (2, 0, 0, 0, 0, 0, 0, 2, 0, 2, 2, 2, 2, 2, 2, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 2, 2, 2, 2, 2, 1, 2, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 2, 2, 2, 2, 2, 1, 2, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 0, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2),
                (2, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
                (2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2))
        self.height = len(self.maze)
        self.width = len(self.maze[0])
        self.scaling = scaling
        self.white = (255,255,255)
        self.red = (0,0,255)
        self.blue = (0,255,0)
        self.black = (0,0,0)
        # Create a black image
        self.img = np.zeros((self.height*scaling,self.width*scaling,3), np.uint8)
        self.beacons = []
        for i in range(self.height):
            for j in range(self.width):
                if (self.maze[i][j]==2):
                    self.beacons.append([i,j])


    def drawMaze(self):
        for i in range(self.height):
            for j in range(self.width):
                if self.maze[i][j]==0:
                    color = self.white
                elif self.maze[i][j]==1:
                    color = self.black
                elif self.maze[i][j]==2:
                    color = self.black
                cv2.rectangle(self.img,(j*self.scaling,i*self.scaling),((j+1)*self.scaling,(i+1)*self.scaling),color,-1,8,0)

        

    def drawRobot(self,robot,length=20):
        x = int_(robot.xyh()[0]*self.scaling)
        y = int_(robot.xyh()[1]*self.scaling)
        r = math.radians(robot.xyh()[2])
        cv2.arrowedLine(self.img,(y,x),(int(y+length*math.cos(r)),int(x+length*math.sin(r))),self.red,1,8,0)

    def drawParticles(self,particles,length=10):
        for i in range(len(particles)):
            x = int_(particles[i].xyh()[0]*self.scaling)
            y = int_(particles[i].xyh()[1]*self.scaling)
            r = math.radians(particles[i].xyh()[2])
            cv2.arrowedLine(self.img,(y,x),(int(y+length*math.cos(r)),int(x+length*math.sin(r))),self.blue,1,8,0)


    def randomPlace(self,xy="lost"):
        if (xy=="lost"):
            x = random.uniform(0, self.height)
            y = random.uniform(0, self.width)
        else:
            x = xy[0] + np.random.normal(0, 0.5, 1)[0]
            y = xy[1] + np.random.normal(0, 0.5, 1)[0]
        return x, y


    def randomFreeSpace(self,xy="lost"):
        while True:
            x, y = self.randomPlace(xy)
            if self.isFree(x, y):
                return x, y

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def distanceToNearestWall(self, xyh, angleDiff=5):
        d = 99999
        r_x = xyh[0]    
        r_y = xyh[1]
        r_h = xyh[2]
        for beacon in self.beacons:
            c_x = beacon[0]
            c_y = beacon[1]
            deltaY = r_y - c_y
            deltaX = c_x - r_x
            angleInDegrees = (math.atan2(deltaY, deltaX) * 180 / math.pi + 90)%360
            diffInDegrees = min((angleInDegrees-r_h)%360,(r_h-angleInDegrees)%360)
            if (diffInDegrees < angleDiff):
                r = 0
                length = 10
                x = c_x*self.scaling
                y = c_y*self.scaling
                distance = self.distance(c_x, c_y, x, y)
                if distance < d:
                    d = distance
        return d/self.scaling


    def isIn(self, x, y):
        if x < 0 or y < 0 or x > self.height or y > self.width:
            return False
        return True

    def isFree(self, x, y):
        if not self.isIn(x, y):
            return False

        return self.maze[int(x)][int(y)] == 0