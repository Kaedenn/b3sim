#!/usr/bin/env python

"""
Tiny wrapper around the main program, simulation.py.
"""

import os
import sys

def _get_path(f):
    d = os.path.dirname(sys.argv[0])
    return os.path.join(d, f) if d else f

MAIN_PROGRAM = "simulation.py"

execfile(_get_path(MAIN_PROGRAM))
