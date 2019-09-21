#!/usr/bin/env python

"""
"""

import numpy as np
import os
import sys

b3Scalar = float

b3vMzeroMask = (-0.0, -0.0, -0.0, -0.0)
b3v1110 = (1.0, 1.0, 1.0, 0.0)
b3vHalf = (0.5, 0.5, 0.5, 0.5)

def b3MakeVector3(x, y, z, w=0):
  return np.array((x, y, z, w), dtype=np.float32)

def b3MakeVector4(x, y, z, w):
  return np.array((x, y, z, w), dtype=np.float32)

# vim: set ts=2 sts=2 sw=2:
