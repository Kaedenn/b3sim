
"""
Build script for compiling FFI module files (.module)

Module files must be Python dict literals of the following form:
  {
    "name": "mod_name",
    "cdef": "mod_cdef",
    "code": "mod_code",
    "extra_args": ("arg1", "arg2", ...)
    "extra_kwargs": {"arg1": "val1", "arg2": "val2", ...}
  }

name: module name; must be a valid Python module name
cdef: module cdefs; identifiers to be exported
code: module code; must be self-contained C and include necessary files
extra_args: (optional) additional positional arguments to pass to set_source()
extra_kwargs: (optional) additional keyword arguments to pass to set_source()

For an example, see ffi/example.module.
"""

import argparse
import ast
import logging
import os
import sys

LOGGING_FORMAT = "%(levelname)s: %(message)s"
logging.basicConfig(format=LOGGING_FORMAT, level=logging.INFO)
logger = logging.getLogger(__name__)

import cffi
from cffi import FFI

def retab(text):
  "Replaces tabs with eight spaces"
  return text.replace("\t", " "*8)

def compile_module(name, cdef, code, *args, **kwargs):
  "Compile an FFI module into a Python module"
  verbose = False
  if "verbose" in kwargs:
    verbose = kwargs["verbose"]
    del kwargs["verbose"]
  ffibuilder = FFI()
  if cdef:
    ffibuilder.cdef(cdef)
  if verbose:
    sys.stderr.write("Compiling module {}:\n".format(name))
    sys.stderr.write("Module cdefs:\n")
    for lnr, line in enumerate(cdef.splitlines()):
      if len(line.strip()) > 0:
        sys.stderr.write("{:04d}\t{}\n".format(lnr, line))
    sys.stderr.write("Module source:\n")
    for lnr, line in enumerate(code.splitlines()):
      if len(line.strip()) > 0:
        sys.stderr.write("{:04d}\t{}\n".format(lnr, line))
  ffibuilder.set_source(name, code, *args, **kwargs)
  ffibuilder.compile(verbose=verbose)

def parse_module(modfile):
  "Parse .module file, returning its name, code, args, and kwargs"
  module = open(modfile).read()
  moddef = ast.literal_eval(module)
  logger.debug("Parsed module: {!r}".format(moddef))
  mname, mcode = moddef["name"], moddef["code"]
  mcdef = moddef.get("cdef", "")
  margs = moddef.get("extra_args", ())
  mkwargs = moddef.get("extra_kwargs", {})
  return {
    "name": mname,
    "code": mcode,
    "cdef": mcdef,
    "extra_args": margs,
    "extra_kwargs": mkwargs
  }

def special_xkeys_module():
  "Create a special X11/Xutil.h keys module"
  CDEF_LINE_FMT = "const int Key_{key};"
  CODE_LINE_FMT = "const int Key_{key} = {val}; /* XK_{key} */".strip()
  cdef_lines = []
  code_lines = []
  for line in open("/usr/include/X11/keysymdef.h"):
    if line.startswith("#define XK_"):
      tokens = line.strip().split()
      keyname, keyval = tokens[1][3:], tokens[2]
      cdef_line = CDEF_LINE_FMT.format(key=keyname)
      code_line = CODE_LINE_FMT.format(key=keyname, val=keyval)
      cdef_lines.append(cdef_line)
      code_lines.append(code_line)
  return {
    "name": "xkeys",
    "cdef": "\n".join(cdef_lines),
    "code": "\n".join(code_lines),
    "extra_args": (),
    "extra_kwargs": {}
  }

if __name__ == "__main__":
  p = argparse.ArgumentParser(epilog="""
The following special modules are available: "xkeys".
  """)
  p.add_argument("module", nargs="+",
                 help="module to compile (path or special module name)")
  p.add_argument("-v", "--verbose", action="store_true", help="verbose output")
  p.add_argument("-a", "--action", choices=("build", "clean", "distclean", "list"),
                 default="build", help="action to take")
  args = p.parse_args()
  modules = []
  if args.verbose:
    logger.setLevel(logging.DEBUG)
  logger.debug("Parsed arguments: {!r}".format(args))

  # Gather module definitions
  for modfile in args.module:
    if modfile == "xkeys":
      logger.info("Building special xkeys module")
      modules.append(special_xkeys_module())
      continue
    try:
      modules.append(parse_module(modfile))
    except KeyError as e:
      logger.error("Invalid module {}; missing required attribute {}".format(modfile, e))
    except (IOError, ValueError) as e:
      logger.error("Invalid module {}: {}".format(modfile, e))

  # Perform requested action on module definitions
  for module in modules:
    mname = module["name"]
    mcdef = module["cdef"]
    mcode = module["code"]
    margs = module["extra_args"]
    mkwargs = module["extra_kwargs"]
    try:
      if args.action == "build":
        # Compile module
        compile_module(mname, mcdef, mcode, *margs, **mkwargs)
        logger.info("Compiled module {} into {}".format(modfile, mname))
      elif args.action in ("clean", "distclean"):
        # Remove generated module files (and, optionally, compiled shared object)
        logger.info("Removing compiled module {}".format(mname))
        exts = ["o", "c"]
        if args.action == "distclean":
          exts.append("so")
        for ext in exts:
          fname = "{}.{}".format(mname, ext)
          if os.path.exists(fname):
            logger.info("Removing file {}".format(fname))
            os.remove(fname)
      elif args.action == "list":
        # Print module information
        print("{}\t{}\t{!r}".format(modfile, mname, retab(mcode)))
    except (IOError, ValueError) as e:
      logger.error("Invalid module {!r}: {}".format(module, e))
    except cffi.VerificationError as e:
      logger.error("Failed to compile module {}: {}".format(module, e))

# vim: set ts=2 sts=2 sw=2:
