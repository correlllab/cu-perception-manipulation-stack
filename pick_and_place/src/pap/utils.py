from __future__ import division, print_function, absolute_import


def Point2list(point):
    return [point.x, point.y, point.z]


def Quaternion2list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
