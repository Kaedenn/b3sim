#!/usr/bin/env python

"""
Utility functions that don't really belong anywhere else

Axes:
  +X is east
  -X is west
  +Y is south
  -Y is north
  +Z is up
  -Z is down
"""

from pyb3 import pybullet as p
import os
import sys
import numpy as np
import time

try:
  from monotonic import monotonic
except ImportError:
  from time import monotonic

# Color constants and functions {{{0
# Primary and secondary colors
C3_RED = [1, 0, 0]
C3_GRN = [0, 1, 0]
C3_BLU = [0, 0, 1]
C3_YEL = [1, 1, 0]
C3_CYN = [0, 1, 1]
C3_MAG = [1, 0, 1]
C3_BLK = [0, 0, 0]
C3_WHT = [1, 1, 1]

# Colors for specific axes and walls
C_X1, C_X2 = C3_RED, C3_GRN
C_Y1, C_Y2 = C3_BLU, C3_MAG
C_Z1, C_Z2 = C3_CYN, C3_YEL

# Primary and secondary colors at full opacity
C4_RED = [1, 0, 0, 1]
C4_GRN = [0, 1, 0, 1]
C4_BLU = [0, 0, 1, 1]
C4_YEL = [1, 1, 0, 1]
C4_CYN = [0, 1, 1, 1]
C4_MAG = [1, 0, 1, 1]
C4_BLK = [0, 0, 0, 1]
C4_WHT = [1, 1, 1, 1]

def withAlpha(c, a):
  "Apply alpha value to the color"
  return [c[0], c[1], c[2], a]
# 0}}}

# Utility functions for numpy {{{0
def norm(v):
  "Calculate the (Cartesian) length of the vector"
  a = np.array(v)
  return np.sqrt(np.sum(np.power(a, 2)))

def unit(v):
  "Convert v to a unit vector"
  return np.divide(v, norm(v))

def d2r(d):
  "Convert degrees to radians"
  return d / 360 * np.pi
# 0}}}

# Camera/axis constants {{{0

# Axis names
CAM_AXIS_PITCH = "pitch"
CAM_AXIS_YAW = "yaw"
CAM_AXIS_DIST = "distance"
CAM_AXIS_TARGET = "target"
CAM_AXIS_X = "x"
CAM_AXIS_Y = "y"
CAM_AXIS_Z = "z"

# Cardinal direction names
CAM_UP = "+" + CAM_AXIS_Z
CAM_DOWN = "-" + CAM_AXIS_Z
CAM_NORTH = "-" + CAM_AXIS_Y
CAM_SOUTH = "+" + CAM_AXIS_Y
CAM_EAST = "+" + CAM_AXIS_X
CAM_WEST = "-" + CAM_AXIS_X

# Axis vectors
AXIS_X = np.array([1, 0, 0])
AXIS_Y = np.array([0, 1, 0])
AXIS_Z = np.array([0, 0, 1])

# Cardinal direction names to axis vectors
CAM_AXES = {
  CAM_NORTH: -AXIS_Y,
  CAM_SOUTH: +AXIS_Y,
  CAM_EAST: +AXIS_X,
  CAM_WEST: -AXIS_X
}

# Colors for each axis
CAM_AXIS_COLORS = {
  CAM_NORTH: C3_GRN,
  CAM_SOUTH: C3_GRN,
  CAM_EAST: C3_RED,
  CAM_WEST: C3_RED,
  CAM_UP: C3_BLU,
  CAM_DOWN: C3_BLU
}

# Cam axis names to axis vectors
CAM_AXIS_VECTORS = {
  CAM_AXIS_X: AXIS_X,
  CAM_AXIS_Y: AXIS_Y,
  CAM_AXIS_Z: AXIS_Z
}

# 0}}}

def applyAxisMovements(camera=None, **movements): # {{{0
  """
  Apply the axis movements to the camera (absolute movement if camera is None)
  """
  if camera is None:
    camera = {}
  pitch = camera.get(CAM_AXIS_PITCH, np.float(0))
  yaw = camera.get(CAM_AXIS_YAW, np.float(0))
  dist = camera.get(CAM_AXIS_DIST, np.float(0))
  target = camera.get(CAM_AXIS_TARGET, np.zeros(3))

  for axis, movement in movements.items():
    if axis in (CAM_AXIS_X, CAM_AXIS_Y, CAM_AXIS_Z):
      target += movement * CAM_AXIS_VECTORS[axis]
    elif axis == CAM_AXIS_PITCH:
      pitch += np.float(movement)
    elif axis == CAM_AXIS_YAW:
      yaw += np.float(movement)
    elif axis == CAM_AXIS_DIST:
      dist += np.float(movement)
    elif axis == CAM_AXIS_TARGET:
      target += np.array(movement)

  if dist < 0:
    dist = 0

  return {
    CAM_AXIS_PITCH: np.mod(pitch, 360),
    CAM_AXIS_YAW: np.mod(yaw, 360),
    CAM_AXIS_DIST: dist,
    CAM_AXIS_TARGET: target
  }
# 0}}}

def assertSuccess(val, msg): # {{{0
  "Assert val is not -1, then return val"
  if val == -1:
    raise RuntimeError("Bullet API Failure: {}".format(msg))
  return val
# 0}}}

def getRemove(d, key, dflt=None, typeObj=None): # {{{0
  "remove a key from a dict and return its value, optionally applying a type"
  if key in d:
    val = d[key]
    del d[key]
    if typeObj is not None:
      return typeObj(val)
    return val
  return dflt
# 0}}}

def formatVec(v): # {{{0
  "Format a 3-vector or 4-vector as a string"
  return "({})".format(", ".join("{:10.2f}".format(i) for i in v))
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

