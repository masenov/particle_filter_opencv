from __future__ import absolute_import
import random
import math
import bisect
from simulation import Maze
import pdb
import numpy


sigma2 = 0.9 ** 2

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(0.02, *coords)

def add_some_noise(*coords):
    return add_noise(0.1, *coords)

class Particle(object):
    def __init__(self, x, y, heading=None, w=1, noisy=False):
        if heading is None:
            heading = random.uniform(0, 360)
        if noisy:
            x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def xy(self):
        return self.x, self.y

    def xyh(self):
        return (self.x, self.y, self.h)

    @classmethod
    def createRandom(self, particle_count, maze, xy="lost"):
        particles = []
        for i in range(particle_count):
            x,y = maze.randomFreeSpace(xy)
            heading = random.uniform(0, 360)
            particles.append(Particle(x,y,heading))
        return particles

    def readSensor(self, maze):
        """
        Find distance to nearest beacon.
        """
        return maze.distanceToNearestWall(self.xyh())

    def weightGauss(self, a, b):
        error = a - b
        g = math.e ** -(error ** 2 / (2 * sigma2))
        return g

    def advance_by(self, speed, checker=None, noisy=False):
        h = self.h
        if noisy:
            speed, h = add_little_noise(speed, h)
            h += random.uniform(-3, 3) # needs more noise to disperse better
        r = math.radians(h)
        dx = math.sin(r) * speed
        dy = math.cos(r) * speed
        if checker is None or checker(self, dx, dy):
            self.move_by(dx, dy)
            return True
        return False

    def move_by(self, x, y):
        self.x += x
        self.y += y



class Robot(Particle):
    speed = 0.2

    def __init__(self,x,y,heading):
        super(Robot, self).__init__(x,y,heading)
        self.step_count = 0

    def chose_random_direction(self):
        heading = random.uniform(0, 360)
        self.h = heading

    def move(self, maze):
        """
        Move the robot. Note that the movement is stochastic too.
        """
        while True:
            self.step_count += 1
            if self.advance_by(self.speed, noisy=True,
                checker=lambda r, dx, dy: maze.isFree(r.x+dx, r.y+dy)):
                break
            # Bumped into something or too long in same direction,
            # chose random new direction
            self.chose_random_direction()


class WeightedDistribution(object):
    def __init__(self, state):
        accum = 0.0
        self.state = [p for p in state if p.w > 0]
        self.distribution = []
        for x in self.state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None
    