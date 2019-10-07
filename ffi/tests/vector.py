#!/usr/bin/env python

"""
Vector tests
"""

import cffi
import numpy as np
import math
import logging
import operator
import os
import sys
import unittest

LOGGING_FORMAT = "%(levelname)s: %(message)s"
logging.basicConfig(level=logging.DEBUG, format=LOGGING_FORMAT)

# Append the path to the ffi module to sys.path
MODULE_PATH = os.path.dirname(sys.argv[0])
FFI_PATH = os.path.realpath(os.path.join(MODULE_PATH, ".."))
HOST_PATH = os.path.realpath(os.path.join(MODULE_PATH, "../.."))
sys.path.append(FFI_PATH)
sys.path.append(HOST_PATH)

from ffi.vector import Vec, VEC_EPSILON, vec_ffi as ffi, vec_lib as lib

class TestVector(unittest.TestCase):
  def assertEqual(self, v1, v2):
    if isinstance(v1, Vec) or isinstance(v2, Vec):
      return self.assertTrue(v1 == v2)
    if isinstance(v1, np.ndarray) or isinstance(v2, np.ndarray):
      return self.assertTrue(all(v1 == v2))
    return super(TestVector, self).assertEqual(v1, v2)

  def assertApprox(self, v1, v2, eps=VEC_EPSILON):
    return self.assertAlmostEqual(np.linalg.norm(v1), np.linalg.norm(v2), delta=eps)

  def testSize(self):
    self.assertEqual(ffi.sizeof("vec3_t"), 16)
    self.assertEqual(ffi.sizeof("vec4_t"), 16)
    self.assertEqual(ffi.sizeof("m128_t"), 16)
    self.assertEqual(ffi.sizeof("m32_t"), 4)

  def testBasic(self):
    v0 = Vec()
    v3 = Vec(0, 0, 0)
    v4 = Vec(0, 0, 0, 0)
    z3 = np.zeros(3)
    z4 = np.zeros(4)
    t3 = (0, 0, 0)
    t4 = (0, 0, 0, 0)
    self.assertEqual(v0, v3)
    self.assertNotEqual(v0, Vec(z4))
    self.assertEqual(v4, Vec(z4))
    self.assertEqual(tuple(v3), t3)
    self.assertEqual(tuple(v4), t4)
    self.assertEqual(repr(v0), repr(v3))
    v3.x = 123.01
    self.assertApprox(v3, Vec(123.01, 0, 0), VEC_EPSILON * 100)
    v3.x += 100
    self.assertApprox(v3, Vec(223.01, 0, 0), VEC_EPSILON * 200)
    v3.y = 10.1
    self.assertApprox(v3, Vec(223.01, 10.1, 0), VEC_EPSILON * 200)
    v3.z = 1.2
    self.assertApprox(v3, Vec(223.01, 10.1, 1.2), VEC_EPSILON * 200)
    v3.w = 1
    self.assertEqual(len(v3), 4)
    self.assertEqual(np.array(Vec(1, 2, 3)), np.array((1, 2, 3)))
    self.assertEqual(Vec.V100.maxAxis(), 0)
    self.assertEqual(Vec.V010.maxAxis(), 1)
    self.assertEqual(Vec.V001.maxAxis(), 2)
    self.assertEqual(Vec(0, 1, 2).minAxis(), 0)
    self.assertEqual(Vec(1, 0, 2).minAxis(), 1)
    self.assertEqual(Vec(1, 2, 0).minAxis(), 2)

  def testMath(self):
    self.assertEqual(Vec.V100, Vec.V100)
    self.assertEqual(Vec.V010, Vec.V010)
    self.assertEqual(Vec.V001, Vec.V001)
    self.assertEqual(Vec.V100 + Vec.V010, Vec(1, 1, 0))
    self.assertEqual((Vec.V100 + Vec.V010 + Vec.V001) / 2, Vec.VHalf)
    self.assertEqual(2 * Vec.V100 + Vec.V010, Vec(2, 1, 0))
    self.assertEqual(4 * (Vec.V100 + Vec.V010) / 2, Vec(2, 2, 0))
    self.assertApprox(Vec(0.1, 0.2, 0.3), Vec(1.1, 2.2, 3.3) - Vec(1, 2, 3))

  def testBasicAlg(self):
    self.assertEqual(abs(Vec.V000), abs(Vec.V0000))
    self.assertEqual(Vec.V000.norm(), 0)
    self.assertEqual(Vec.V0000.norm(), 0)
    self.assertEqual(Vec.V100.norm(), 1)
    self.assertEqual(Vec.V010.norm(), 1)
    self.assertEqual(Vec.V001.norm(), 1)
    self.assertEqual(Vec.V100.norm(), 1)
    self.assertEqual(Vec.V010.norm(), 1)
    self.assertEqual(Vec.V001.norm(), 1)
    self.assertEqual(Vec.V100.cross(Vec.V100), Vec.V000)
    self.assertEqual(Vec.V100.cross(Vec.V010), Vec.V001)
    self.assertEqual(Vec.V100.dot(Vec.V100), 1)
    self.assertEqual(Vec.V100.dot(Vec.V010), 0)
    self.assertEqual(Vec.V100.unit(), Vec.V100)
    self.assertEqual(Vec.V010.unit(), Vec.V010)
    self.assertEqual(Vec.V001.unit(), Vec.V001)

  def testAlg(self):
    self.assertApprox(Vec(1, 0, 0).distance(Vec(0, 0, 1)), 2**0.5)
    self.assertEqual((-Vec.V100).distance(Vec.V100), 2)
    self.assertEqual(Vec(1, 1, 0).distance(Vec(1, 1, 1)), 1)
    self.assertEqual(Vec(1, 1, 1, 0).distance(Vec(1, 1, 1, 1)), 1)
    self.assertEqual(Vec(-1, -1, -1).absolute(), Vec.V111)
    self.assertApprox(Vec(1, 1, 1).unit().norm(), 1)
    self.assertEqual(Vec(0, 0, 0).safe_unit(), None)

  def testCyl(self):
    self.assertApprox(Vec.V100.get_cyl(), (1, 0, 0))
    self.assertApprox(Vec.V010.get_cyl(), (1, np.math.pi/2, 0))
    self.assertApprox(Vec.V001.get_cyl(), (1, np.math.pi/2, 1))
    self.assertApprox((2 * Vec.V100).get_cyl(), (2, 0, 0))
    self.assertApprox((2 * Vec.V010).get_cyl(), (2, np.math.pi/2, 0))
    self.assertApprox((2 * Vec.V001).get_cyl(), (2, np.math.pi/2, 2))

  def testSph(self):
    self.assertApprox(Vec.V100.get_sph(), (1, 0, np.math.pi/2))
    self.assertApprox((2 * Vec.V100).get_sph(), (2, 0, np.math.pi/2))
    # TODO: more

  def testTransform3D(self):
    v1 = Vec(1, 1, 1)
    m1 = np.identity(len(v1), dtype=Vec.dtype)
    self.assertEqual(v1.transform(m1), v1)
    self.assertEqual(v1.transform(m1 * 2), v1 * 2)
    v1.itransform(m1 + 2 * m1)
    self.assertEqual(v1, Vec(3, 3, 3))
    # TODO: more

  def testTransform4D(self):
    v1 = Vec.V1111
    m1 = np.identity(len(v1), dtype=Vec.dtype)
    self.assertEqual(v1.transform(m1), v1)
    self.assertEqual(v1.transform(m1 * 2), v1 * 2)
    v1.itransform(m1 + 2 * m1)
    self.assertEqual(v1, Vec(3, 3, 3, 3))
    # TODO: more

if __name__ == "__main__":
  unittest.main()

