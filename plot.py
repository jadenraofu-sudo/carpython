import numpy as np
import matplotlib.pyplot as plt
import math
from config import environment, R, LX, LY, SEC_PER_STEP
from helper import linehelper, semihelper, tankturnhelper, wheelspeeds


coord = np.array((0.0, 0.0))
# `direction` is an angle in radians; 0 points to the right (positive x)
direction = 0.0
wheels = np.array((0, 0, 0, 0))

xbounds = [-5, 7]
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
    if environment == 'mac':
        axd['left'].plot(x, y)
    return np.array((tdx, tdy))


# draws a semi circle between the points coord and (h, k).
# It chooses between the two possible semicircles using dir
def semi(point, dir, v=1.0, w=0.0, align=True, tangential=True):
    global coord
    global direction
    point_arr = np.array(point)
    h, k = point
    radius = np.linalg.norm(coord - point_arr) / 2
    if tangential:
        if dir > 0:
            w = np.rad2deg(v/radius)
        else:
            w = -np.rad2deg(v/radius)
    # Align to initial tangent direction if requested
    if align:
        start_ang = (np.atan2(k - coord[1], h - coord[0]) +  np.pi)
        new_heading = dir * np.array((-np.sin(start_ang), np.cos(start_ang)))
        time_turn, dir_turn, ccw = tankturnhelper(
            coord, direction, new_heading
        )
        for t, d in zip(time_turn, dir_turn):
            direction = d
            draw_arrow()
            plot_wheels([0, 0], ccw * w)
            plt.pause(t)

    # Arc movement using semihelper
    points, time, dirs, unit_vecs = semihelper(
        coord, direction, point_arr, dir, v, w, tangential
    )

    for pts, t, d, uv in zip(points, time, dirs, unit_vecs.T):
        simulate(uv * v, w, t)
        print("distance to theoretical point: ", np.linalg.norm(pts - coord))
        draw_arrow()
        plot_wheels(uv * v, w)
        plt.pause(t)


# draws a segment between coord and (h, k)
def seg(h, k):
    global coord
    axd['left'].plot([coord[0], h], [coord[1], k])
    coord = np.array((h, k))


# draws lines between the points saved in corners starting at the start point
def line(point, v=1.0, w=0.0, align=True):
    global direction
    global coord
    unit_vec = point - coord
    unit_vec = unit_vec / np.linalg.norm(unit_vec)
    if align:
        time, dir, ccw = tankturnhelper(coord, direction, unit_vec)
    for t, d in zip(time, dir):
        direction = d
        draw_arrow()
        plot_wheels([0, 0], ccw * w)
        plt.pause(t)
    points, time, dir, unit_vec = linehelper(coord, direction, point, v, w)
    vec = v * unit_vec
    for pts, t, d in zip(points, time, dir):
        simulate(vec, w, t)
        print("distance to theoretical point: ", np.linalg.norm(pts - coord))
        draw_arrow()
        plot_wheels(vec, w)
        plt.pause(t)

def simulate(v, w, t):
    global direction
    global coord
    speed = np.linalg.norm(v)
    w_rad = math.radians(w)
    if w == 0:
        dx = v[0] * t
        dy = v[1] * t
    else:
        dx = speed/w_rad * (math.sin(direction + w_rad * t) - math.sin(direction))
        dy = speed/w_rad * (math.cos(direction) - math.cos(direction + w_rad * t))

    direction += w_rad * t
    coord += np.array([dx, dy])
