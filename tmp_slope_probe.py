import math


def slope_y_pos(hx, hy, x, uphill, floor_top):
    left = -hx
    right = hx
    bottom = -hy
    top = hy
    ratio = (top - bottom) / (right - left)
    if left < x:
        d = x - right
        y = top + d * ratio if uphill else bottom - d * ratio
    else:
        d = left - x
        y = top + d * ratio if not uphill else bottom - d * ratio
    return y


def world_y(player_x, player_y, uphill):
    cx, cy = 105.0, 90.0
    rot_deg = -90.0
    sx, sy = 1.0, -1.0
    hx, hy = 30.0, 15.0
    player_half = 15.0
    gravity_down = True
    th = math.radians(rot_deg)
    dx = player_x - cx
    dy = player_y - cy
    rx = dx * math.cos(th) + dy * math.sin(th)
    ry = -dx * math.sin(th) + dy * math.cos(th)
    px = rx / sx
    py = ry / sy
    ysurf = slope_y_pos(hx, hy, px, uphill, gravity_down)
    ang = math.atan2(hy, hx)
    rad = player_half / math.cos(ang)
    py_land = ysurf + rad
    wxr = px * sx
    wyr = py_land * sy
    wy = cy + wxr * math.sin(th) + wyr * math.cos(th)
    return px, py, ysurf, wy


for x in [100, 105, 110, 115]:
    for uphill in [True, False]:
        px, py, ys, wy = world_y(x, 76.0, uphill)
        print(f"x={x} uphill={uphill} px={px:.2f} py={py:.2f} ys={ys:.2f} wy={wy:.2f}")
