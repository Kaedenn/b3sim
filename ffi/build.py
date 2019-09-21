
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

The following special modules are available:
  xkeys   Export a significant number of Key_* constants, from X11/keysymdef.h.

Special modules are generated upon request, rather than from a module file.
This allows far more flexibility in generating a large amount of identifiers.
"""

import argparse
import ast
import logging
import os
import sys

import cffi
FFI = cffi.FFI

LOGGING_FORMAT = "%(levelname)s: %(message)s"
logging.basicConfig(format=LOGGING_FORMAT, level=logging.INFO)
logger = logging.getLogger(__name__)

def get_remove(d, k, dflt=None):
  "Obtain and remove a key from a dict"
  if k in d:
    v = d[k]
    del d[k]
    return v
  return dflt

def retab(text):
  "Replaces tabs with eight spaces"
  return text.replace("\t", " "*8)

def compile_module(name, cdef, code, *args, **kwargs):
  "Compile an FFI module into a Python module"
  verbose = get_remove(kwargs, "verbose", False)
  tmpdir = get_remove(kwargs, "tmpdir", os.getcwd())
  ffibuilder = FFI()
  if cdef:
    ffibuilder.cdef(cdef)
  logger.debug("Compiling module {}:".format(name))
  logger.debug("Module cdefs:")
  for lnr, line in enumerate(cdef.splitlines()):
    if len(line.strip()) > 0:
      logger.debug("{:04d}\t{}".format(lnr, line))
  logger.debug("Module source:")
  for lnr, line in enumerate(code.splitlines()):
    if len(line.strip()) > 0:
      logger.debug("{:04d}\t{}".format(lnr, line))
  ffibuilder.set_source(name, code, *args, **kwargs)
  ffibuilder.compile(verbose=verbose, tmpdir=tmpdir)

def parse_module(modfile):
  "Parse .module file, returning name, code, args, and kwargs"
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
  ACTIONS = ("build", "clean", "distclean", "list")
  p.add_argument("module", nargs="+", help="module to compile (path or special module name)")
  p.add_argument("-a", "--action", choices=ACTIONS, default="build", help="action to take")
  p.add_argument("-v", "--verbose", action="store_true", help="verbose output")
  p.add_argument("-t", default=os.path.dirname(sys.argv[0]), help="temporary directory")
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
      logger.exception(e)
      logger.error("Invalid module {}; missing required attribute {}".format(modfile, e))
    except (IOError, ValueError) as e:
      logger.exception(e)
      logger.error("Invalid module {}: {}".format(modfile, e))

  # Perform requested action on module definitions
  for module in modules:
    mname = module["name"]
    mcdef = module["cdef"]
    mcode = module["code"]
    margs = module["extra_args"]
    mkwargs = module["extra_kwargs"]
    logger.debug("Processing module {}...".format(mname))
    try:
      if args.action == "build":
        # Compile module
        logger.debug("Compiling module {}...".format(mname))
        compile_module(mname, mcdef, mcode, tmpdir=args.t, *margs, **mkwargs)
        logger.info("Compiled module {} into {}".format(modfile, mname))
        # Move compiled objects into cwd
        for ext in ("a", "dll", "dylib", "so"):
          mfile = "{}.{}".format(mname, ext)
          oldname = os.path.join(args.t, mfile)
          newname = mfile
          if os.path.exists(oldname):
            logger.info("Moving module {} to {}".format(oldname, newname))
            os.rename(oldname, newname)
      elif args.action in ("clean", "distclean"):
        # Remove generated module files (and, optionally, compiled shared object)
        logger.info("Removing compiled module {} intermediate files".format(mname))
        for ext in ("c", "o"):
          fname = os.path.join(args.t, "{}.{}".format(mname, ext))
          if os.path.exists(fname):
            logger.info("Removing file {}".format(fname))
            os.remove(fname)
        if args.action == "distclean":
          logger.info("Removing compiled module {}".format(mname))
          for ext in ("a", "dll", "dylib", "so"):
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
