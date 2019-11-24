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

Possible Vector128 dtype:
d = np.dtype((np.float128, np.dtype({"names": list("xyzw"), "formats": [np.float32]*4})))
a = np.array([0], dtype=d)
a[0] = <float128 value>
a['x'][0] = <float32 x coordinate>

"""

from pyb3 import pybullet as p
import argparse
import errno
import math
import os
import random
import re
import subprocess
import sys
import numpy as np
import time

try:
  from monotonic import monotonic
except ImportError:
  from time import monotonic

# Custom argparse help formatter {{{0
class ArgumentDefaultsHelpFormatter(argparse.HelpFormatter):
  """Help message formatter which adds default values to argument help.
  This formatter acts like the standard ArgumentDefaultsHelpFormatter except
  for omitting default values of True, False, or None.
  """
  def _get_help_string(self, action):
    help = action.help
    if '%(default)' not in action.help:
      if action.default not in (argparse.SUPPRESS, True, False, None):
        defaulting_nargs = [argparse.OPTIONAL, argparse.ZERO_OR_MORE]
        if action.option_strings or action.nargs in defaulting_nargs:
          help += ' (default: %(default)s)'
    return help
  def _fill_text(self, text, width, indent):
    return "".join([indent + line for line in text.splitlines(True)])
# 0}}}

# Color constants {{{0
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
C_NONE = [0, 0, 0, 0]
# 0}}}

def parseColor(c, cmax=256, scale=1, forceAlpha=None): # {{{0
  """Parse a color 0 <= c < cmax and rescale to 0 <= c < scale

  Understood inputs:
    string "#rrggbb"
    string "#rrggbbaa"
    string "rr,gg,bb"
    string "rr,gg,bb,aa"
    3-sequence (r, g, b) (can be any iterable such that len(i) == 3)
    4-sequence (r, g, b, a) (can be any iterable such that len(i) == 4)

  If forceAlpha is supplied, then the existing alpha component is discarded
  and forceAlpha used instead.

  Returns a 4-tuple of floats c such that 0 <= c < scale.
  """
  r = g = b = a = float(cmax)
  if isinstance(c, basestring):
    if c[0] == '#' and len(c) in (7, 9):
      r = int(c[1:3], 16)
      g = int(c[3:5], 16)
      b = int(c[5:7], 16)
      if len(c) == 9:
        a = int(c[7:9], 16)
    elif c.count(",") in (2, 3):
      parts = map(float, c.split(","))
      r = parts[0]
      g = parts[1]
      b = parts[2]
      if len(parts) == 4:
        a = parts[3]
    else:
      # Invalid format; return None
      return None
  else:
    try:
      if len(c) in (3, 4):
        r, g, b = c[0], c[1], c[2]
        if len(c) == 4:
          a = c[3]
    except TypeError as e:
      # Unexpected type; return None
      return None
  if any(x is None for x in (r, g, b, a)):
    # Failed parsing; return None
    return None
  cr = float(r) / cmax * scale
  cg = float(g) / cmax * scale
  cb = float(b) / cmax * scale
  if forceAlpha is not None:
    ca = float(forceAlpha)
  else:
    ca = float(a) / cmax * scale
  return cr, cg, cb, ca
# 0}}}

def withAlpha(c, a): # {{{0
  "Apply alpha value to the color"
  return [c[0], c[1], c[2], a]
# 0}}}

def unit(v): # {{{0
  "Convert v to a unit vector"
  return np.divide(v, np.linalg.norm(v))
# 0}}}

def V3(x=0, y=0, z=0): # {{{0
  "Create a 3-dimensional vector"
  return np.array((x, y, z))
# 0}}}

def V4(x=0, y=0, z=0, w=0): # {{{0
  "Create a 4-dimensional vector"
  return np.array((x, y, z, w))
# 0}}}

def randomVec(low=-0.5, high=0.5): # {{{0
  "Create a vector with each component having a random value"
  return V3(
    random.random() * (high - low) + low,
    random.random() * (high - low) + low,
    random.random() * (high - low) + low
  )
# 0}}}

# Axes, coordinates, vectors, etc {{{0

# Basis vectors and their combinations
V3_X = V3(1, 0, 0)
V3_Y = V3(0, 1, 0)
V3_Z = V3(0, 0, 1)
V3_XY = V3(1, 1, 0)
V3_XZ = V3(1, 0, 1)
V3_YZ = V3(0, 1, 1)
V3_XYZ = V3(1, 1, 1)

# Camera axis names
CAM_AXIS_PITCH = "pitch"
CAM_AXIS_YAW = "yaw"
CAM_AXIS_DIST = "distance"
CAM_AXIS_TARGET = "target"
CAM_AXIS_X = "x"
CAM_AXIS_Y = "y"
CAM_AXIS_Z = "z"

# Camera cardinal direction names
CAM_UP = "+" + CAM_AXIS_Z
CAM_DOWN = "-" + CAM_AXIS_Z
CAM_NORTH = "-" + CAM_AXIS_Y
CAM_SOUTH = "+" + CAM_AXIS_Y
CAM_EAST = "+" + CAM_AXIS_X
CAM_WEST = "-" + CAM_AXIS_X

# Map cardinal direction names to axis vectors
CAM_AXES = {
  CAM_NORTH: -V3_Y,
  CAM_SOUTH: +V3_Y,
  CAM_EAST: +V3_X,
  CAM_WEST: -V3_X,
  CAM_UP: +V3_Z,
  CAM_DOWN: -V3_Z
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
  CAM_AXIS_X: V3_X,
  CAM_AXIS_Y: V3_Y,
  CAM_AXIS_Z: V3_Z
}
# 0}}}

def parseCameraSpec(spec, dfp, dfy, dfd, dft): # {{{0
  """
  Parse a comma-separated string of values as a camera position and orientation
  """
  t = spec.split(",")
  pitch, yaw, dist, target = dfp, dfy, dfd, dft
  def getTok(idx, tp, dflt):
    if len(t) > idx:
      if t[idx] == 'Df':
        return dflt
      try:
        return tp(t[idx])
      except ValueError:
        pass
    return dflt
  pitch = getTok(0, float, pitch)
  yaw = getTok(1, float, yaw)
  dist = getTok(2, float, dist)
  target[0] = getTok(3, float, target[0])
  target[1] = getTok(4, float, target[1])
  target[2] = getTok(5, float, target[2])
  return pitch, yaw, dist, target
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
  "Remove a key from a dict and return its value, optionally applying a type"
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

def openFileDialog(title=None, globs=(), multiple=False): # {{{0
  "Create an open-file dialog using zenity"
  args = ["zenity", "--file-selection"]
  if title is not None:
    args.append("--title={}".format(title))
  if globs:
    args.append("--file-filter={}".format(" ".join(globs)))
  if multiple:
    args.append("--multiple")
  args.append("--separator=:")
  try:
    return subprocess.check_output(args).rstrip("\r\n").split(":")
  except subprocess.CalledProcessError as e:
    if e.returncode == 1:
      # No files selected
      return ()
    raise
  except OSError as e:
    if e.errno == errno.ENOENT:
      # Not having zenity is a warning
      sys.stderr.write("Failed invoking {!r}; is zenity installed?\n".format(args))
      return ()
    raise
# 0}}}

def centerString(s, width=80, ch=" ", padSpace=False): # {{{0
  """Pad a string to center it within a given width.
  s         The string to center
  width     The desired with of the resulting string (default: 80)
  ch        The padding character (default: space)
  padSpace  If True, add a space before and after s
  """
  if padSpace:
    s = " " + s + " "
  padSize = (width - len(s)) / len(ch) / 2
  val = (ch * padSize) + s + (ch * padSize)
  if len(val) < width:
    val += ch * ((width - len(val)) / len(ch))
  return val
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

