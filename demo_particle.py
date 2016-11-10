from __future__ import absolute_import
import random
import math
import bisect
from simulation import Maze
from particle_filter import *
import cv2
import copy
import time

PARTICLE_COUNT = 200    # Total number of particles

ROBOT_HAS_COMPASS = False # Does the robot know where north is? If so, it
# makes orientation a lot easier since it knows which direction it is facing.
# If not -- and that is really fascinating -- the particle filter can work
# out its heading too, it just takes more particles and more time. Try this
# with 3000+ particles, it obviously needs lots more hypotheses as a particle
# now has to correctly match not only the position but also the heading.





world = Maze(0)


# initial distribution assigns each particle an equal probability
robbie = Robot(3,5,180)
world.particles = Particle.createRandom(PARTICLE_COUNT, world, robbie.xy())

world.drawMaze()
backup_img = copy.copy(world.img)
world.drawRobot(robbie)
world.drawParticles(world.particles)



cv2.imshow("img",world.img)
cv2.waitKey(0)




while True:
    # Read robbie's sensor
    r_d = robbie.readSensor(world)
    
    print r_d
    # Update particle weight according to how good every particle matches
    # robbie's sensor reading
    

    for p in world.particles:
        if world.isFree(p.xy()[0],p.xy()[1]):
            p_d = p.readSensor(world)
            p.w = p.weightGauss(r_d, p_d)
        else:
            p.w = 0

    # ---------- Try to find current best estimate for display ----------
    
    # ---------- Show current state ----------
    #world.show_particles(particles, maze_data, m_x, m_y, m_confident, robbie)
    #world.show_mean(m_x, m_y, m_confident)
    #world.show_robot(robbie)

    # ---------- Shuffle particles ----------
    new_particles = []

    # Normalise weights
    nu = sum(p.w for p in world.particles)
    if nu:
        for p in world.particles:
            p.w = p.w / nu
    
    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(world.particles)
    
    for _ in world.particles:
        p = dist.pick()
        if p is None:  # No pick b/c all totally improbable
            new_particle = Particle.createRandom(1, world)[0]
            #import pdb; pdb.set_trace()
        else:
            new_particle = Particle(p.x, p.y,
                    heading=robbie.h if ROBOT_HAS_COMPASS else p.h,
                    noisy=True)
        new_particles.append(new_particle)

    world.particles = new_particles

    # ---------- Move things ----------
    old_heading = robbie.h
    robbie.move(world)
    d_h = robbie.h - old_heading
    # Move particles according to my belief of movement (this may
    # be different than the real movement, but it's all I got)
    for p in world.particles:
        p.h += d_h # in case robot changed heading, swirl particle heading too
        p.advance_by(robbie.speed)

    world.drawRobot(robbie)
    world.drawParticles(world.particles)


    print "test"
    cv2.imshow("img",world.img)
    cv2.waitKey(0)
    #time.sleep(1)
    #cv2.destroyWindow("img")
    world.img = copy.copy(backup_img)