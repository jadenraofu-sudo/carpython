import numpy as np
import matplotlib.pyplot as plt
import math
from config import R, LX, LY

SEC_PER_STEP = 0.1
# STEP_SIZE = SEC_PER_STEP * V
# TURN_STEP = 30.0  # degrees per second
coord = np.array((0, 0))
# `direction` is an angle in radians; 0 points to the right (positive x)
direction = 0.0
wheels = np.array((0, 0, 0, 0))

xbounds = [-5, 5]
ybounds = [-5, 5]

plt.style.use('_mpl-gallery-nogrid')
fig, axd = plt.subplot_mosaic(
    [
        ['left', 'left', 'upper left', 'upper right'],
        ['left', 'left', 'lower left', 'lower right'],
    ],
    figsize=(8, 4),
    layout="constrained",
)
axd['left'].set_xlim(xbounds[0], xbounds[1])
axd['left'].set_ylim(ybounds[0], ybounds[1])
axd['left'].plot(1, 1)
fig.suptitle('robot path')


def wheelspeeds(v, w):
    wfl = 1 / R * (v[0] - v[1] - w * (LX + LY))
    wfr = 1 / R * (v[0] + v[1] + w * (LX + LY))
    wrl = 1 / R * (v[0] + v[1] - w * (LX + LY))
    wrr = 1 / R * (v[0] - v[1] + w * (LX + LY))
    return np.array((wfl, wfr, wrl, wrr))


def plot_wheels(v_world, w_deg_per_s):
    """Plot wheel contact direction vectors and numeric spin speeds.

    v_world: 2-vector linear velocity in world frame (m/s)
    w_deg_per_s: angular velocity of the robot (deg/s)
    """
    # map axes to wheels: front-left, front-right, rear-left, rear-right
    axes = ['upper left', 'upper right', 'lower left', 'lower right']
    wheel_pos = [(LX, LY), (LX, -LY), (-LX, LY), (-LX, -LY)]
    # convert world velocity into robot frame
    # direction is an angle (radians)
    theta = float(direction)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    Rmat = np.array([[cos_t, sin_t], [-sin_t, cos_t]])
    v_robot = Rmat.dot(v_world)
    w_rad = math.radians(w_deg_per_s)
    ws = wheelspeeds(v_robot, w_rad)

    # for each wheel compute contact-point velocity:
    # v_contact = v_robot + omega x r
    for ax_name, pos, w_val in zip(axes, wheel_pos, ws):
        ax = axd[ax_name]
        ax.cla()
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_aspect('equal')
        x, y = pos
        # rotational component at wheel (robot frame): omega x r = [-w*y, w*x]
        v_rot = np.array([-w_rad * y, w_rad * x])
        v_contact = v_robot + v_rot
        # draw contact velocity vector in robot frame (origin is wheel contact)
        # map robot-frame (x, y) -> plot-frame (-y, x) so forward (1, 0) is up
        plot_x, plot_y = -v_contact[1], v_contact[0]
        ax.quiver(0, 0, plot_x, plot_y, angles='xy', scale_units='xy',
                  scale=1, color='tab:blue', pivot='mid')

        # also show wheel spin magnitude (rad/s)
        ax.text(0.1, -1.2, f"{w_val:.2f} rad/s", fontsize=9)
        ax.set_title(ax_name)


def draw_arrow():
    dx, dy = math.cos(direction), math.sin(direction)
    axd['left'].quiver(coord[0], coord[1], dx, dy, pivot='mid', angles='xy')


def tankturn(newdir, w=30.0):
    global direction
    turnstep = w * SEC_PER_STEP
    # ang1/ang2 in degrees for stepping;
    # accept newdir as angle (degrees) or 2-vector
    ang1 = math.degrees(float(direction))
    # convert newdir to degrees from x-axis
    # if it's a vector, otherwise treat as scalar
    if hasattr(newdir, '__len__') and len(newdir) == 2:
        ang2 = math.degrees(math.atan2(newdir[1], newdir[0]))
    else:
        ang2 = float(newdir)
    angdiff = (ang2 - ang1) % 360
    if angdiff > 180:
        angdiff -= 360
    numsteps = int(abs(angdiff) // (turnstep))
    angstep = (turnstep) * np.sign(angdiff)
    for i in range(numsteps):
        ang = ang1 + angstep * (i + 1)
        # set direction angle (radians) and draw via draw_arrow()
        direction = math.radians(ang)
        draw_arrow()
        plot_wheels([0, 0], w)
        plt.pause(SEC_PER_STEP)
    remaining = abs(angdiff) - numsteps * turnstep
    if remaining > 1e-9:
        # final partial-step: set direction and draw
        direction = math.radians(ang2)
        draw_arrow()
        plot_wheels([0, 0], w)
        plt.pause(remaining / (turnstep) * SEC_PER_STEP)
    # store direction as an angle (radians)
    direction = math.radians(ang2)


# draws an arc of radius r centered at (h, k) starting from angle start_ang
# in the direction dir (1 for ccw, -1 for cw)
def arc(r, h, k, start_ang, ang, dir, tangential):
    global direction
    global coord
    if dir > 0:
        theta = np.linspace(np.deg2rad(start_ang),
                            np.deg2rad(start_ang + ang), 100)
    else:
        theta = np.linspace(np.deg2rad(start_ang),
                            np.deg2rad(start_ang - ang), 100)
    x = h + r * np.cos(theta)
    y = k + r * np.sin(theta)
    # tangent vector at arc end (robot frame)
    tdx = dir * (-1 * np.sin(theta[-1]))
    tdy = dir * (np.cos(theta[-1]))
    if tangential:
        direction = math.atan2(tdy, tdx)
    coord = np.array([x[-1], y[-1]])
    axd['left'].plot(x, y)
    return np.array((tdx, tdy))


# draws a semi circle between the points coord and (h, k).
# It chooses between the two possible semicircles using dir
def semi(point, dir, v=1.0, w=0.0, align=True, tangential=True):
    global coord
    global direction
    h, k = point
    if tangential:
        w = 0.0
    step_size = v * SEC_PER_STEP
    start_ang = np.atan2(k - coord[1], h - coord[0]) * 180 / np.pi + 180
    if align:
        tankturn(dir * np.array((-np.sin(start_ang), np.cos(start_ang))))
    end = np.array((h, k))
    radius = np.linalg.norm(coord - end) / 2
    centerx = (coord[0] + h) / 2
    centery = (coord[1] + k) / 2
    numseg = int(radius * np.pi // step_size)
    angchange = step_size / (radius * np.pi) * 180
    for i in range(numseg):
        if dir > 0:
            unit_vec = arc(radius, centerx, centery,
                           angchange * i + start_ang,
                           angchange, dir, tangential)
        else:
            unit_vec = arc(radius, centerx, centery,
                           (-1 * angchange) * i + start_ang,
                           angchange, dir, tangential)
        direction += math.radians(w) * SEC_PER_STEP
        draw_arrow()
        plot_wheels(unit_vec * v, w)
        plt.pause(SEC_PER_STEP)
    lastang = 180 - angchange * numseg
    dist = radius * np.deg2rad(lastang)
    if dist > 1e-9:
        if dir > 0:
            unit_vec = arc(radius, centerx, centery,
                           start_ang + numseg * angchange,
                           lastang, dir, tangential)
        else:
            unit_vec = arc(radius, centerx, centery,
                           start_ang - numseg * angchange,
                           lastang, dir, tangential)
        direction += math.radians(w) * dist / step_size * SEC_PER_STEP
        draw_arrow()
        plot_wheels(unit_vec * v, w)
        plt.pause(dist / step_size * SEC_PER_STEP)
    coord = np.array((h, k))


# draws a segment between coord and (h, k)
def seg(h, k):
    global coord
    axd['left'].plot([coord[0], h], [coord[1], k])
    coord = np.array((h, k))


# draws lines between the points saved in corners starting at the start point
def line(point, v=1.0, w=0.0, align=True):
    global direction
    global coord
    h, k = point
    step_size = v * SEC_PER_STEP
    end = np.array((h, k))
    dist = np.linalg.norm(coord - end)
    vec = end - coord
    unit_vec = vec / dist
    if align:
        tankturn(unit_vec)
    numseg = int(dist // step_size)
    points = [coord + i * step_size * unit_vec for i in range(numseg + 1)]
    points.pop(0)
    # linear speed corresponding to step_size
    speed = step_size / SEC_PER_STEP
    for [x, y] in points:
        seg(x, y)
        # update heading for this time step (direction is angle)
        rad = float(direction)
        rad += math.radians(w) * SEC_PER_STEP
        direction = rad
        # plot robot arrow and wheel states for this motion step
        draw_arrow()
        plot_wheels(unit_vec * speed, w)
        plt.pause(SEC_PER_STEP)
    dist = np.linalg.norm(coord - end)
    if dist > 1e-9:
        seg(h, k)
        # remaining motion for the final sub-step (update angle)
        rad = float(direction)
        dt = dist/step_size * SEC_PER_STEP
        rad += math.radians(w) * dt
        direction = rad
        draw_arrow()
        # plot wheels for the remaining motion (use same speed)
        plot_wheels(unit_vec * speed, w)
        plt.pause(dt)
    coord = end
