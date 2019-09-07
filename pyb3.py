#!/usr/bin/env python

"""
Wrapper module for pybullet

Provides seamless APIs between different versions of the pybullet API.

Recommended usage is one of the following:
  from pyb3 import pybullet
  from pyb3 import pybullet as p
"""

# pybullet TODO:
# Support "Button" and "ComboBox" debug inputs
#   Requires extensive changes to pybullet and PhysicsServerExample.cpp
# Add the following functions/macros:
#   getCameraPosition
#   Bullet3Common:
#     b3Printf
#     b3Warning
#     b3Error
#   PhysicsServerExample.cpp
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
# Separate physics simulation from user input?
#   Determine if pybullet ever releases the GIL

__all__ = ["pybullet"]

import pybullet
pybullet._PYB3_POLYFILLED = True

# Ensure pybullet has the expected properties {{{0
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
# 0}}}

# vim: set ts=2 sts=2 sw=2 et:

