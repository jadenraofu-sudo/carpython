import numpy as np
from config import environment, R, LX, LY, SEC_PER_STEP
import math


def wheelspeeds(v, w):
    wfl = 1 / R * (v[0] - v[1] - w * (LX + LY))
    wfr = 1 / R * (v[0] + v[1] + w * (LX + LY))
    wrl = 1 / R * (v[0] + v[1] - w * (LX + LY))
    wrr = 1 / R * (v[0] - v[1] + w * (LX + LY))
    return np.array((wfl, wfr, wrl, wrr))


def wheelhelper(direction, v_world, w_deg_per_s):
    # convert world velocity into robot frame
    # direction is an angle (radians)
    theta = float(direction)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    Rmat = np.array([[cos_t, sin_t], [-sin_t, cos_t]])
    v_robot = Rmat.dot(v_world)
    w_rad = math.radians(w_deg_per_s)
    ws = wheelspeeds(v_robot, w_rad)
    return ws


def tankturnhelper(cur, curdir, newdir, w=30.0):
    ang1 = np.rad2deg(float(curdir))
    if hasattr(newdir, '__len__') and len(newdir) == 2:
        ang2 = math.degrees(math.atan2(newdir[1], newdir[0]))
    else:
        ang2 = float(newdir)
    angdiff = (ang2 - ang1) % 360
    ccw = 1
    if angdiff > 180:
        angdiff -= 360
        ccw = -1
    turnstep = w * SEC_PER_STEP * np.sign(angdiff)
    numsteps = int(abs(angdiff) // abs(turnstep))
    dir = [curdir + i * math.radians(turnstep) for i in range(1, numsteps + 1)]
    time = [SEC_PER_STEP] * numsteps
    if environment == 'mac':
        points = [cur] * numsteps
    if abs(dir[-1] - math.radians(ang2)) > 1e-6:
        dir.append(math.radians(ang2))
        time.append(abs(SEC_PER_STEP * (abs(angdiff) -
                                        abs(dir[-2] - curdir)) / turnstep))
        if environment == 'mac':
            points.append(cur)
    wheelspeeds_list = [wheelhelper(d, np.array((0, 0)),
                                    np.sign(angdiff) * w) for d in dir]
    if environment == 'mac':
        return time, dir, ccw
    else:
        return wheelspeeds_list, time


def linehelper(cur, curdir, point, v=1.0, w=0.0):
    h, k = point
    step_size = v * SEC_PER_STEP
    dist = np.linalg.norm(point - cur)
    unit_vec = (point - cur) / dist
    numseg = int(dist // step_size)
    if environment == 'mac':
        points = [cur + i * step_size * unit_vec for i in range(1, numseg + 1)]
    time = [SEC_PER_STEP] * numseg
    dir = [curdir + i * math.radians(w) *
           SEC_PER_STEP for i in range(1, numseg + 1)]
    if ((cur + numseg * step_size * unit_vec) != [h, k]).any():
        dt = (dist - numseg * step_size) / v
        if environment == 'mac':
            points.append([h, k])
        time.append(dt)
        dir.append(curdir + numseg * math.radians(w) *
                   SEC_PER_STEP + math.radians(w) * dt)
    wheelspeeds_list = [wheelhelper(d, unit_vec * v, w) for d in dir]
    if environment == 'mac':
        return points, time, dir, unit_vec
    else:
        return wheelspeeds_list, time


def semihelper(cur, curdir, point, dir, v=1.0, w=0.0, tangential=True):
    h, k = point
    step_size = v * SEC_PER_STEP
    radius = np.linalg.norm(cur - point) / 2
    centerx = (cur[0] + h) / 2
    centery = (cur[1] + k) / 2
    numseg = int(radius * np.pi // step_size)
    angchange = step_size / (radius)
    start_ang = (np.atan2(k - cur[1], h - cur[0]) + np.pi)
    if dir > 0:
        if tangential:
            w = np.rad2deg(v/radius)
        theta = [start_ang + i * angchange for i in range(1, numseg + 1)]
    else:
        if tangential:
            w = -np.rad2deg(v/radius)
        theta = [start_ang - i * angchange for i in range(1, numseg + 1)]
    time = [SEC_PER_STEP] * numseg
    theta.append(np.atan2(k - cur[1], h - cur[0]))
    time.append((radius * np.pi - numseg * step_size) / v)
    tdx = dir * (-1 * np.sin(theta))
    tdy = dir * (np.cos(theta))
    if tangential:
        direction = [math.atan2(tdy[i], tdx[i]) for i in range(len(theta))]
    else:
        direction = [curdir] * len(theta)
        direction += math.radians(w) * SEC_PER_STEP
    direction = np.array(direction)
    points = [np.array((centerx + radius * math.cos(t),
                        centery + radius * math.sin(t))) for t in theta]
    unit_vec = np.array((tdx, tdy))
    wheelspeeds_list = [wheelhelper(d, uv * v, w)
                        for (d, uv) in zip(direction, unit_vec.T)]
    if environment == 'mac':
        return points, time, direction, unit_vec
    else:
        return wheelspeeds_list, time
