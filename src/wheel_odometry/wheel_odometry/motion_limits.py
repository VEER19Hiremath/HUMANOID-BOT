"""Shared motion limits for odometry and map bounds."""

MAP_ODOM_DX = 1.6
MAP_ODOM_DY = 1.26
MAP_X_MIN = -5.85
MAP_X_MAX = 5.5
MAP_Y_MIN = -3.5
MAP_Y_MAX = 4.2

MIN_TRAVEL_M = 0.01


def map_pose_from_odom(odom_x, odom_y):
    return odom_x + MAP_ODOM_DX, odom_y + MAP_ODOM_DY


def inside_map(odom_x, odom_y):
    mx, my = map_pose_from_odom(odom_x, odom_y)
    return MAP_X_MIN <= mx <= MAP_X_MAX and MAP_Y_MIN <= my <= MAP_Y_MAX


def wheel_travel(left_m, right_m, live_threshold=MIN_TRAVEL_M):
    if left_m < live_threshold and right_m >= live_threshold:
        return right_m
    if right_m < live_threshold and left_m >= live_threshold:
        return left_m
    return 0.5 * (left_m + right_m)
