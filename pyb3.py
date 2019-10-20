#!/usr/bin/env python

"""
Wrapper module for pybullet

Provides seamless APIs between different versions of the pybullet API

Recommended usage is one of the following:
  from pyb3 import pybullet
  from pyb3 import pybullet as p
"""

# pybullet TODO: {{{0
# Add custom logging handler calling b3Print, b3Warning, b3Error
# Implement --verbose in both the client and server (half done)
# Support "Button" debug inputs (done)
# Support "ComboBox" debug inputs
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
#   b3Print (doesn't log to example browser)
#   b3Warning (doesn't log to example browser)
#   b3Error (doesn't log to example browser)
#   b3ReferenceFrameHelper.hpp
#     getPointWorldToLocal
#     getPointLocalToWorld
#     getAxisWorldToLocal
#     getAxisLocalToWorld
#     getTransformWorldToLocal
#     getTransformLocalToWorld
# Incorporate keypress logic directly?
#   Provide simpler API for keyboard camera movement?
#   Remove all built-in keybinds?
#     ../examples/SharedMemory/PhysicsServerExample.cpp
#   Support adding/changing keybinds in PhysicsServerExample?
# 0}}}

# pybullet documentation (information discovered so far): {{{0
#
###############################################################################
#
# pybullet.GUI:
#   calls InProcessPhysicsClientSharedMemory(argc, argv, true)
#   defined in examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp
#   * argv[0]: --unused
#   * argv[1]: --start_demo_name=Physics Server
#     calls btCreateInProcessExampleBrowser
#     defined in examples/ExampleBrowser/InProcessExampleBrowser.cpp
#       constructs btInProcessExampleBrowserInternalData
#     dispatches B3_THREAD_SCHEDULE_TASK
#       ../examples/MultiThreading/b3PosixThreadSupport.cpp
#
#   then calls PhysicsClientSharedMemory::connect()
#     allocates shared memory
#   and we're done
#
# Return value is opaque pointer to PhysicsClientSharedMemory C-cast to a pointer
# to `struct <name>_ { int dummy; }`
#
# GUI example name and descriptions:
#   gDefaultExamplesPhysicsServer
#   in examples/ExampleBrowser/InProcessExampleBrowser.cpp
#
# PhysicsServerExample is constructed in
#   examples/SharedMemory/PhysicsServerExample.cpp
# Uses btSoftBodyDynamicsWorld
#
# Commands are handled in
#   examples/SharedMemory/PhysicsServerCommandProcessor.cpp
#
# Keybinds are handled in
#   examples/ExampleBrowser/OpenGLExampleBrowser.cpp
#
###############################################################################
#
# PhysicsServerExample:
#   m_guiHelper: GUIHelperInterface
#   GUIHelperInterface::m_multiThreadedHelper: MultiThreadedOpenGLGuiHelper
#
# 0}}}

import ctypes
import logging
import os
import random
import sys
import pybullet
pybullet._PYB3_POLYFILL = True

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

# Import cffi.FFI
try:
  from cffi import FFI
  HAVE_FFI = True
except ImportError:
  HAVE_FFI = False

# Global logger
LOGGING_FORMAT = "%(filename)s:%(lineno)s:%(levelname)s: %(message)s"
logging.basicConfig(format=LOGGING_FORMAT, level=logging.INFO)
_logger = None
def getLogger(*args, **kwargs):
  "Get the global logger"
  global _logger
  if _logger is None:
    _logger = logging.getLogger("simulation")
  return _logger

# Provide convenience access to libc
try:
  libc = ctypes.cdll.LoadLibrary("libc.so.6")
except OSError:
  libc = ctypes.cdll.msvcrt

# Ensure pybullet has expected properties {{{0
def _ensurePybulletProperty(name, val):
  "Ensure pybullet has the given attribute with the given value"
  if not hasattr(pybullet, name):
    setattr(pybullet, name, val)
  elif getattr(pybullet, name) != val:
    print("pybullet[{}] != {}".format(name, val))

_ensurePybulletProperty("MOUSE_BUTTON_EVENT", 2)
_ensurePybulletProperty("MOUSE_LB", 0)
_ensurePybulletProperty("MOUSE_MOVE_EVENT", 1)
_ensurePybulletProperty("MOUSE_PRESS", 3)
_ensurePybulletProperty("MOUSE_RB", 2)
_ensurePybulletProperty("MOUSE_RELEASE", 4)
_ensurePybulletProperty("MOUSE_WHEEL", 1)
_ensurePybulletProperty("B3G_UP", pybullet.B3G_UP_ARROW)
_ensurePybulletProperty("B3G_LEFT", pybullet.B3G_LEFT_ARROW)
_ensurePybulletProperty("B3G_DOWN", pybullet.B3G_DOWN_ARROW)
_ensurePybulletProperty("B3G_RIGHT", pybullet.B3G_RIGHT_ARROW)
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

del _ensurePybulletProperty

# b3Print
if not hasattr(pybullet, "b3Print"):
  def b3Print(msg):
    "Polyfilled b3Print function implemented in pure Python"
    sys.stdout.write(msg)
    sys.stdout.write("\n")
  pybullet.b3Print = b3Print

# b3Warning
if not hasattr(pybullet, "b3Warning"):
  def b3Warning(msg):
    sys.stderr.write("Warning:")
    sys.stderr.write(msg)
    sys.stdout.write("\n")
  pybullet.b3Warning = b3Warning

# b3Error
if not hasattr(pybullet, "b3Error"):
  def b3Error(msg):
    sys.stderr.write("Error:")
    sys.stderr.write(msg)
    sys.stdout.write("\n")
  pybullet.b3Error = b3Error

# NotConnectedError
if not hasattr(pybullet, "NotConnectedError"):
  pybullet.NotConnectedError = pybullet.error

# addUserDebugButton
if not hasattr(pybullet, "addUserDebugButton"):
  def addUserDebugButton(*args, **kwargs):
    "Stub function for an unimplemented feature"
    raise NotImplementedError()
  pybullet.addUserDebugButton = addUserDebugButton

# readUserDebugButton
if not hasattr(pybullet, "readUserDebugButton"):
  def readUserDebugButton(*args, **kwargs):
    "Stub function for an unimplemented feature"
    raise NotImplementedError()
  pybullet.readUserDebugButton = readUserDebugButton

# resetUserDebugButton
if not hasattr(pybullet, "resetUserDebugButton"):
  def resetUserDebugButton(*args, **kwargs):
    "Stub function for an unimplemented feature"
    raise NotImplementedError
  pybullet.resetUserDebugButton = resetUserDebugButton
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

# vim: set ts=2 sts=2 sw=2 et:

