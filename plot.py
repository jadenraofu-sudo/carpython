import numpy as np
import matplotlib.pyplot as plt
import math

SEC_PER_STEP = 0.2
STEP_SIZE = 1.0
TURN_SPEED = 15.0  # degrees per second
coord = np.array((0, 0))
# direction is now stored as an angle in radians (0 = +x, pi = -x)
direction = np.pi  # pointing left (-1, 0)


def tankturn(newdir, turnspeed = TURN_SPEED):
    global direction
    newdir_norm = np.linalg.norm(newdir)
    if newdir_norm < 1e-9:
        return
    desired = np.arctan2(newdir[1], newdir[0])
    # smallest signed angle difference in [-pi, pi]
    diff = (desired - direction + np.pi) % (2 * np.pi) - np.pi
    step = np.deg2rad(turnspeed)
    numseg = int(abs(diff) // step)
    sign = np.sign(diff) if diff != 0 else 0
    for i in range(numseg):
        direction += sign * step
        # normalize to [-pi, pi]
        direction = (direction + np.pi) % (2 * np.pi) - np.pi
        dx, dy = np.cos(direction), np.sin(direction)
        if np.isfinite(dx) and np.isfinite(dy) and (abs(dx) + abs(dy) > 1e-12):
            plt.quiver(coord[0], coord[1], dx, dy, pivot='mid', angles='xy')
        plt.pause(SEC_PER_STEP)
    # final small remaining rotation
    rem = abs(diff) - numseg * step
    if rem > 1e-9:
        direction += sign * rem
        direction = (direction + np.pi) % (2 * np.pi) - np.pi
        dx, dy = np.cos(direction), np.sin(direction)
        if np.isfinite(dx) and np.isfinite(dy) and (abs(dx) + abs(dy) > 1e-12):
            plt.quiver(coord[0], coord[1], dx, dy, pivot='mid', angles='xy')
        plt.pause(SEC_PER_STEP)
    # snap to exact desired angle
    direction = desired

# draws an arc of radius r centered at (h, k) starting from angle start_ang
# in the direction dir (1 for ccw, -1 for cw)
def arc(r, h, k, start_ang, ang, dir):
    if dir > 0:
        theta = np.linspace(np.deg2rad(start_ang),
                            np.deg2rad(start_ang + ang), 300)
    else:
        theta = np.linspace(np.deg2rad(start_ang),
                            np.deg2rad(start_ang - ang), 300)
    x = h + r * np.cos(theta)
    y = k + r * np.sin(theta)
    vec = np.array((-r * np.sin(theta[-1]), r * np.cos(theta[-1])))
    den = np.linalg.norm(vec)
    if den < 1e-9:
        dx, dy = 0.0, 0.0
    else:
        ang = np.arctan2(vec[1], vec[0])
        dx, dy = np.cos(ang), np.sin(ang)
    if np.isfinite(dx) and np.isfinite(dy) and (abs(dx) + abs(dy) > 1e-12):
        plt.quiver(x[-1], y[-1], dx, dy, pivot='mid', angles='xy')
    plt.plot(x, y)
    return [x[-1], y[-1]]


# draws a semi circle between the points coord and (h, k).
# It chooses between the two possible semicircles using dir
def semi(h, k, dir, step_size = STEP_SIZE):
    global coord
    start_ang = np.atan2(k - coord[1], h - coord[0])*180/np.pi + 180
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
            arc(radius, centerx, centery, start_ang + numseg * angchange, lastang, dir)
        else:
            arc(radius, centerx, centery, start_ang - numseg * angchange, lastang, dir)
        plt.pause(dist/STEP_SIZE * SEC_PER_STEP)
    coord = np.array((h, k))


# draws a segment between coord and (h, k)
def seg(h, k):
    global coord
    x = np.array((coord[0], h))
    y = np.array((coord[1], k))
    plt.plot(x, y)
    coord = np.array((h, k))
    dx, dy = np.cos(direction), np.sin(direction)
    if np.isfinite(dx) and np.isfinite(dy) and (abs(dx) + abs(dy) > 1e-12):
        plt.quiver(coord[0], coord[1], dx, dy, pivot='mid', angles='xy')


# draws lines between the points saved in corners starting at the start point
def line(corners, step_size = STEP_SIZE):
    global coord
    global direction
    for [h, k] in corners:
        newdir = np.array((h - coord[0], k - coord[1]))
        newdir_norm = np.linalg.norm(newdir)
        if newdir_norm < 1e-9:
            # no movement needed
            continue
        tankturn(newdir)
        end = np.array((h, k))
        dist = np.linalg.norm(coord - end)
        if dist < 1e-9:
            continue
        vec = end - coord
        # set global direction as angle
        direction = np.arctan2(vec[1], vec[0])
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


def curvedquad(corners, step_size = STEP_SIZE):
    global coord
    t = 0
    for [h, k] in corners:
        if t % 2 == 1:
            semi(h, k, 1)
        else:
            line([[h, k]], step_size)
        coord = np.array((h, k))
        t += 1
