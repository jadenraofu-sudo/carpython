import numpy as np
import matplotlib.pyplot as plt
import math

SEC_PER_STEP = 1.0
STEP_SIZE = 1.0
TURN_STEP = 30.0  # degrees per second
coord = np.array((0, 0))
direction = np.array((1, 0))


def tankturn(newdir, turnstep=TURN_STEP):
    global direction
    ang1 = math.atan2(direction[1], direction[0]) * 180 / math.pi
    ang2 = math.atan2(newdir[1], newdir[0]) * 180 / math.pi
    angdiff = (ang2 - ang1 + 360) % 360
    if angdiff > 180:
        angdiff -= 360
    numsteps = int(abs(angdiff) // (turnstep))
    angstep = (turnstep) * np.sign(angdiff)
    for i in range(numsteps):
        ang = ang1 + angstep * (i + 1)
        dx, dy = np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))
        plt.quiver(coord[0], coord[1], dx, dy, pivot='mid', angles='xy')
        plt.pause(SEC_PER_STEP)
    remaining = abs(angdiff) - numsteps * turnstep
    if remaining > 1e-9:
        ang = ang1 + angdiff
        dx, dy = np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))
        plt.quiver(coord[0], coord[1], dx, dy, pivot='mid', angles='xy')
        plt.pause(remaining / (turnstep) * SEC_PER_STEP)
    direction = newdir


# draws an arc of radius r centered at (h, k) starting from angle start_ang
# in the direction dir (1 for ccw, -1 for cw)
def arc(r, h, k, start_ang, ang, dir):
    global direction
    if dir > 0:
        theta = np.linspace(np.deg2rad(start_ang),
                            np.deg2rad(start_ang + ang), 300)
    else:
        theta = np.linspace(np.deg2rad(start_ang),
                            np.deg2rad(start_ang - ang), 300)
    x = h + r * np.cos(theta)
    y = k + r * np.sin(theta)
    direction = dir*np.array((-r * np.sin(theta[-1]), r * np.cos(theta[-1])))
    dx, dy = direction
    if np.isfinite(dx) and np.isfinite(dy) and (abs(dx) + abs(dy) > 1e-12):
        plt.quiver(x[-1], y[-1], dx, dy, pivot='mid', angles='xy')
    plt.plot(x, y)


# draws a semi circle between the points coord and (h, k).
# It chooses between the two possible semicircles using dir
def semi(h, k, dir, step_size=STEP_SIZE):
    global coord
    start_ang = np.atan2(k - coord[1], h - coord[0])*180/np.pi + 180
    tankturn(dir*np.array((-np.sin(start_ang), np.cos(start_ang))))
    end = np.array((h, k))
    radius = np.linalg.norm(coord - end)/2
    centerx = (coord[0] + h)/2
    centery = (coord[1] + k)/2
    numseg = int(radius*np.pi // step_size)
    angchange = step_size / (radius * np.pi) * 180
    for i in range(numseg):
        if dir > 0:
            arc(radius, centerx, centery,
                angchange * i + start_ang, angchange, dir)
        else:
            arc(radius, centerx, centery,
                (-1 * angchange) * i + start_ang, angchange, dir)
        plt.pause(SEC_PER_STEP)
    lastang = 180-angchange * numseg
    dist = radius * np.deg2rad(lastang)
    if dist > 1e-9:
        if dir > 0:
            arc(radius, centerx, centery,
                start_ang + numseg * angchange, lastang, dir)
        else:
            arc(radius, centerx, centery,
                start_ang - numseg * angchange, lastang, dir)
        plt.pause(dist/STEP_SIZE * SEC_PER_STEP)
    coord = np.array((h, k))


# draws a segment between coord and (h, k)
def seg(h, k):
    global coord
    x = np.array((coord[0], h))
    y = np.array((coord[1], k))
    plt.plot(x, y)
    coord = np.array((h, k))
    rad = np.arctan2(direction[1], direction[0])
    dx, dy = np.cos(rad), np.sin(rad)
    if np.isfinite(dx) and np.isfinite(dy) and (abs(dx) + abs(dy) > 1e-12):
        plt.quiver(coord[0], coord[1], dx, dy, pivot='mid', angles='xy')


# draws lines between the points saved in corners starting at the start point
def line(corners, step_size=STEP_SIZE):
    for [h, k] in corners:
        end = np.array((h, k))
        dist = np.linalg.norm(coord - end)
        if dist < 1e-9:
            continue
        vec = end - coord
        tankturn(vec)
        v = vec / dist
        numseg = int(dist // step_size)
        points = [coord + i * step_size * v for i in range(numseg + 1)]
        for [x, y] in points:
            seg(x, y)
            plt.pause(SEC_PER_STEP)
        dist = np.linalg.norm(coord - end)
        if dist > 1e-9:
            seg(h, k)
            plt.pause(dist/STEP_SIZE * SEC_PER_STEP)


def curvedquad(corners, step_size=STEP_SIZE):
    global coord
    t = 0
    for [h, k] in corners:
        if t % 2 == 1:
            semi(h, k, 1)
        else:
            line([[h, k]], step_size)
        coord = np.array((h, k))
        t += 1
