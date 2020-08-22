#!/usr/bin/env python

"""
Wrapper module for pybullet

Provides seamless integration between different versions of the pybullet API

Recommended usage is one of the following:
  from pyb3 import pybullet
  from pyb3 import pybullet as p
"""

# pybullet TODO: {{{0
# Provide a Makefile for compiling pybullet plugins
# Python logging handler calling b3Print, b3Warning, b3Error
# Implement --verbose in both the client and server (half done)
# Support "ComboBox" debug inputs
#
# Support pruning of bodies very far from the origin?
#
# Add the following APIs:
#   Bullet3Common/b3Matrix3x3.h (numpy can replace)
#   Bullet3Common/b3MinMax.h (implement in pure Python)
#   Bullet3Common/b3Quaternion.h (numpy can replace)
#   Bullet3Common/b3Scalar.h (numpy can replace)
#   Bullet3Common/b3Transform.h (numpy can replace most)
#   Bullet3Common/b3TransformUtil.h (numpy can replace most)
#   Bullet3Common/b3Vector3.h (numpy can replace most)
#   LinearMath/btAabbUtil2.h
#   LinearMath/btConvexHull.h
#   LinearMath/btGeometryUtil.h
#   LinearMath/btMatrixX.h (numpy can replace)
#   b3ReferenceFrameHelper.hpp
#     getPointWorldToLocal
#     getPointLocalToWorld
#     getAxisWorldToLocal
#     getAxisLocalToWorld
#     getTransformWorldToLocal
#     getTransformLocalToWorld
#
# Revisit keypress-handling logic:
#   Incorporate KeyPressManager logic in either the physics client or server?
#     Support adding/changing keybinds in PhysicsServerExample?
#     Remove all built-in keybinds?
#       ../examples/SharedMemory/PhysicsServerExample.cpp
#   Provide simpler API for keyboard camera movement?
# 0}}}

import ctypes
import functools
import logging
import os
import random
import sys
import pybullet
pybullet._PYB3_POLYFILL = True

# Determine if cffi is available for use
try:
  from cffi import FFI
  HAVE_FFI = True
except ImportError:
  HAVE_FFI = False

# Provide convenience access to libc
try:
  libc = ctypes.cdll.LoadLibrary("libc.so.6")
except OSError:
  libc = ctypes.cdll.msvcrt

# Extra pybullet modules {{{0
# Grab pybullet_data
try:
  import pybullet_data
  HAS_PYB3_DATA = True
except ImportError as e:
  HAS_PYB3_DATA = False

# Grab pybullet_envs
try:
  import pybullet_envs
  HAS_PYB3_ENVS = True
except ImportError as e:
  HAS_PYB3_ENVS = False

# Grab pybullet_utils
try:
  import pybullet_utils
  HAS_PYB3_UTILS = True
except ImportError as e:
  HAS_PYB3_UTILS = False
# 0}}}

# Global logger {{{0
LOGGING_FORMAT = "%(filename)s:%(lineno)s:%(levelname)s: %(message)s"
logging.basicConfig(format=LOGGING_FORMAT, level=logging.INFO)
_loggers = {}
def getLogger(name="simulation"):
  "Get the named logger, creating it if necessary"
  if name not in _loggers:
    _loggers[name] = logging.getLogger("name")
  return _loggers[name]
# 0}}}

def funcToString(func, *args, **kwargs): # {{{0
  "Format a string representation of the function and arguments"
  try:
    name = func.__name__
  except AttributeError as e:
    name = str(func)
  strs = []
  for i in args:
    strs.append(repr(i))
  for k,v in kwargs.items():
    strs.append("{}={!r}".format(k, v))
  return "{}({})".format(name, ", ".join(strs))
# 0}}}

def getRandomUrdf(): # {{{0
  "Return the path to a random URDF file, if pybullet_data is available"
  if HAS_PYB3_DATA:
    path = os.path.join(pybullet_data.getDataPath(), "random_urdfs")
    if os.path.exists(path):
      nr = random.choice([x for x in os.listdir(path) if x.isdigit()])
      return os.path.join(path, nr, nr + ".urdf")
    else:
      pybullet.b3Error("Path does not exist: {!r}".format(path))
  else:
    pybullet.b3Error("pybullet_data not available")
# 0}}}

def addLogging(func): # {{{0
  "Add logging to the function"
  @functools.wraps(func)
  def wrapper(*args, **kwargs):
    value = func(*args, **kwargs)
    getLogger().info("{} = {}".format(funcToString(func, *args, **kwargs), value))
    return value
  return wrapper
# 0}}}

# Added features {{{0
FEATURE_VERBOSE = "VerboseMode"
FEATURE_NOTCONNECTEDERROR = "NotConnectedError"
FEATURE_X11 = "X11"
FEATURE_B3PRINT = "B3Print"
FEATURE_B3WARNING = "B3Warning"
FEATURE_B3ERROR = "B3Error"
FEATURE_EXTENDEDKEYS = "ExtendedKeySupport"
FEATURE_GUI_BUTTONS = "GUI::Buttons"
FEATURE_GUI_BUTTONS_RESET = "GUI::Buttons::Reset"
FEATURE_SOFT_BODY_EXTRACONFIG = "SoftBody::ExtraConfig"
# 0}}}

# Ensure pybullet has all of the APIs added by the custom build, within
# reason. This is to ensure compatability between the base pybullet API and
# the custom build.
def ensurePybulletAPIs(): # {{{0
  "Ensure pybullet has all of the newly-added constants and functions"
  def _ensurePybulletProperty(name, val): # {{{1
    "Ensure pybullet has the given attribute with the given value"
    logger = getLogger()
    if hasattr(pybullet, name):
      v = getattr(pybullet, name)
      if v != val:
        logger.warning("pybullet[{!r}] == {} (not {})".format(name, v, val))
    setattr(pybullet, name, val)
  # 1}}}

  # Ensure pybullet has the added constants
  _ensurePybulletProperty("MOUSE_MOVE_EVENT", 1)
  _ensurePybulletProperty("MOUSE_BUTTON_EVENT", 2)
  _ensurePybulletProperty("MOUSE_PRESS", 3)
  _ensurePybulletProperty("MOUSE_RELEASE", 4)
  _ensurePybulletProperty("MOUSE_BUTTON_NONE", -1)
  _ensurePybulletProperty("MOUSE_BUTTON_LEFT", 0)
  _ensurePybulletProperty("MOUSE_BUTTON_RIGHT", 2)
  _ensurePybulletProperty("MOUSE_LB", pybullet.MOUSE_BUTTON_LEFT)
  _ensurePybulletProperty("MOUSE_WHEEL", 1)
  _ensurePybulletProperty("MOUSE_RB", pybullet.MOUSE_BUTTON_RIGHT)
  _ensurePybulletProperty("B3G_LEFT", pybullet.B3G_LEFT_ARROW)
  _ensurePybulletProperty("B3G_UP", pybullet.B3G_UP_ARROW)
  _ensurePybulletProperty("B3G_RIGHT", pybullet.B3G_RIGHT_ARROW)
  _ensurePybulletProperty("B3G_DOWN", pybullet.B3G_DOWN_ARROW)

  # Add the rest of the keys using the xkeys FFI module, if available
  try:
    import xkeys
    _ensurePybulletProperty("B3G_KP_HOME", xkeys.lib.Key_KP_Home)
    _ensurePybulletProperty("B3G_KP_LEFT", xkeys.lib.Key_KP_Left)
    _ensurePybulletProperty("B3G_KP_UP", xkeys.lib.Key_KP_Up)
    _ensurePybulletProperty("B3G_KP_RIGHT", xkeys.lib.Key_KP_Right)
    _ensurePybulletProperty("B3G_KP_DOWN", xkeys.lib.Key_KP_Down)
    _ensurePybulletProperty("B3G_KP_END", xkeys.lib.Key_KP_End)
    _ensurePybulletProperty("B3G_KP_PGUP", xkeys.lib.Key_KP_Page_Up)
    _ensurePybulletProperty("B3G_KP_PGDN", xkeys.lib.Key_KP_Page_Down)
    _ensurePybulletProperty("B3G_KP_END", xkeys.lib.Key_KP_End)
    _ensurePybulletProperty("B3G_KP_ENTER", xkeys.lib.Key_KP_Enter)
    _ensurePybulletProperty("B3G_KP_INS", xkeys.lib.Key_KP_Insert)
    _ensurePybulletProperty("B3G_KP_DEL", xkeys.lib.Key_KP_Delete)
    _ensurePybulletProperty("B3G_KP_0", pybullet.B3G_KP_0)
    _ensurePybulletProperty("B3G_KP_1", pybullet.B3G_KP_1)
    _ensurePybulletProperty("B3G_KP_2", pybullet.B3G_KP_2)
    _ensurePybulletProperty("B3G_KP_3", pybullet.B3G_KP_3)
    _ensurePybulletProperty("B3G_KP_4", pybullet.B3G_KP_4)
    _ensurePybulletProperty("B3G_KP_5", pybullet.B3G_KP_5)
    _ensurePybulletProperty("B3G_KP_6", pybullet.B3G_KP_6)
    _ensurePybulletProperty("B3G_KP_7", pybullet.B3G_KP_7)
    _ensurePybulletProperty("B3G_KP_8", pybullet.B3G_KP_8)
    _ensurePybulletProperty("B3G_KP_9", pybullet.B3G_KP_9)
  except ImportError as e:
    getLogger().exception(e)
    _ensurePybulletProperty("B3G_KP_HOME", 0xff95)
    _ensurePybulletProperty("B3G_KP_LEFT", 0xff96)
    _ensurePybulletProperty("B3G_KP_UP", 0xff97)
    _ensurePybulletProperty("B3G_KP_RIGHT", 0xff98)
    _ensurePybulletProperty("B3G_KP_DOWN", 0xff99)
    _ensurePybulletProperty("B3G_KP_PGUP", 0xff9a)
    _ensurePybulletProperty("B3G_KP_PGDN", 0xff9b)
    _ensurePybulletProperty("B3G_KP_END", 0xff9c)
    _ensurePybulletProperty("B3G_KP_ENTER", 0xff8d)
    _ensurePybulletProperty("B3G_KP_INS", 0xff9e)
    _ensurePybulletProperty("B3G_KP_DEL", 0xff9f)
    _ensurePybulletProperty("B3G_KP_0", pybullet.B3G_KP_INS)
    _ensurePybulletProperty("B3G_KP_1", pybullet.B3G_KP_END)
    _ensurePybulletProperty("B3G_KP_2", pybullet.B3G_KP_DOWN)
    _ensurePybulletProperty("B3G_KP_3", pybullet.B3G_KP_PGDN)
    _ensurePybulletProperty("B3G_KP_4", pybullet.B3G_KP_LEFT)
    _ensurePybulletProperty("B3G_KP_5", 0xff9d)
    _ensurePybulletProperty("B3G_KP_6", pybullet.B3G_KP_RIGHT)
    _ensurePybulletProperty("B3G_KP_7", pybullet.B3G_KP_HOME)
    _ensurePybulletProperty("B3G_KP_8", pybullet.B3G_KP_UP)
    _ensurePybulletProperty("B3G_KP_9", pybullet.B3G_KP_PGUP)

  # Add the remaining pybullet functions and types

  if not hasattr(pybullet, "b3Print"): # {{{1
    def b3Print(msg):
      "Polyfilled b3Print function implemented in pure Python"
      sys.stdout.write(msg)
      sys.stdout.write("\n")
    pybullet.b3Print = b3Print
  # 1}}}

  if not hasattr(pybullet, "b3Warning"): # {{{1
    def b3Warning(msg):
      sys.stderr.write("Warning:")
      sys.stderr.write(msg)
      sys.stdout.write("\n")
    pybullet.b3Warning = b3Warning
  # 1}}}

  if not hasattr(pybullet, "b3Error"): # {{{1
    def b3Error(msg):
      sys.stderr.write("Error:")
      sys.stderr.write(msg)
      sys.stdout.write("\n")
    pybullet.b3Error = b3Error
  # 1}}}

  if not hasattr(pybullet, "NotConnectedError"): # {{{1
    pybullet.NotConnectedError = pybullet.error
  # 1}}}

  if not hasattr(pybullet, "addUserDebugButton"): # {{{1
    def addUserDebugButton(*args, **kwargs):
      "Stub function for an unimplemented feature"
      raise NotImplementedError()
    pybullet.addUserDebugButton = addUserDebugButton
  # 1}}}

  if not hasattr(pybullet, "readUserDebugButton"): # {{{1
    def readUserDebugButton(*args, **kwargs):
      "Stub function for an unimplemented feature"
      raise NotImplementedError()
    pybullet.readUserDebugButton = readUserDebugButton
  # 1}}}

  if not hasattr(pybullet, "resetUserDebugButton"): # {{{1
    def resetUserDebugButton(*args, **kwargs):
      "Stub function for an unimplemented feature"
      raise NotImplementedError
    pybullet.resetUserDebugButton = resetUserDebugButton
  # 1}}}

  if not hasattr(pybullet, "loadSoftBody"): # {{{1
    def loadSoftBody(*args, **kwargs):
      "Stub function for an unimplemented feature"
      raise NotImplementedError
    pybullet.loadSoftBody = loadSoftBody
  # 1}}}

  if not hasattr(pybullet, "createSoftBody"): # {{{1
    def createSoftBody(*args, **kwargs):
      "Stub function for an unimplemented feature"
      raise NotImplementedError
    pybullet.createSoftBody = createSoftBody
  # 1}}}
# 0}}}

# Ensure all added functions and attributes exist
ensurePybulletAPIs()
# Don't export the function
del ensurePybulletAPIs

# vim: set ts=2 sts=2 sw=2 et:

