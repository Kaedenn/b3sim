#!/bin/bash

# pybullet3 setup script: downloads, builds, and installs bullet3 and pybullet.

T=0
F=1

export GIT_URL="https://github.com/kaedenn/bullet3"
export BULLET_DIR="$PWD/bullet3"
export BULLET_BUILD_DIR="build_cmake"
export DOT_PYTHON="$HOME/.python"

b3dir="$BULLET_DIR"
b3bdir="$BULLET_BUILD_DIR"
clone=
clone_args=
build=1
pydir="$DOT_PYTHON"
pyinit=
verbose=
dry_run=1
force=

info() {
  echo "$@" >&2
}

warning() {
  echo "WARNING: $@" >&2
}

error() {
  echo "ERROR: $@" >&2
  exit 1
}

checked() {
  if [[ -z "$dry_run" ]]; then
    echo "Executing $@..." >&2
    $@
  else
    echo "DRY RUN: $@ (in $PWD)"
  fi
  status=$?
  if [[ $status -ne 0 ]]; then
    error "$@ exited with non-zero status $status"
  fi
}

usage_exit() {
  cat <<EOF >&2
Usage: $0 [-h] [-c] [-d DIR] [-m DIR] [-b] [-p DIR] [-i] [-g ARGS] [-v] [-n] [-F]

Options:
  -h        show help
  -c        clone bullet3 from github (implies -b)
  -d DIR    use DIR for bullet3; if -c, clone bullet3 into DIR (default: $BULLET_DIR)
  -m NAME   use NAME for the bullet3 build directory name (default: $BULLET_BUILD_DIR)
  -b        compile pybullet3 using cmake and make
  -p DIR    use DIR instead of $DOT_PYTHON
  -i        create symbolic links to pybullet in $DOT_PYTHON
  -g ARGS   pass ARGS to git clone
  -v        verbose/debug output
  -n        dry run: display what would be done without doing it
  -F        force: continue even if files exist (DANGEROUS!)
EOF
  exit 0
}

debug() {
  if [[ -n "$verbose" ]]; then
    echo "DEBUG: $@" >&2
  fi
}

while getopts "hcd:m:bp:ig:vnF" optval; do
  case "$optval" in
    h) usage_exit;;
    c) clone=1; build=1;;
    d) b3dir="$OPTARG";;
    m) b3bdir="$OPTARG";;
    b) build=1;;
    p) pydir="$OPTARG";;
    i) pyinit=1;;
    g) clone_args="$OPTARG";;
    v) verbose=1;;
    n) dry_run=1;;
    F) force=1;;
    ?) error "Invalid option";;
  esac
done

debug "Parsed arguments:"
debug "b3dir: $b3dir"
debug "b3 build dir: $b3bdir"
debug "clone: $clone"
debug "clone args: $clone_args"
debug "pyinit: $pyinit"
debug "build: $build"
debug "pydir: $pydir"
debug "force: $force"

if [[ -n "$clone" ]]; then
  if [[ -d "$b3dir" ]]; then
    info "$b3dir exists: performing git pull instead of clone..."
    (
      cd "$b3dir"
      checked git pull
    )
  else
    checked git clone "$GIT_DIR" "$b3dir" $clone_args
  fi
fi

if [[ ! -d "$b3dir" ]]; then
  error "$b3dir: not a directory"
fi

if [[ ! -f "$b3dir/CMakeLists.txt" ]]; then
  error "CMakeLists.txt not found in $b3dir; is the directory correct?"
fi

if [[ -n "$build" ]]; then
  if [[ -d "$b3dir/$b3bdir" ]]; then
    info "$b3dir/$b3bdir exists; using"
    (
      cd "$b3dir/$b3bdir"
      checked make $make_args
    )
  else
    info "Building bullet3 in $b3dir..."
    (
      checked mkdir "$b3dir/$b3bdir"
      cd "$b3dir/$b3bdir"
      checked cmake ..
      checked make $make_args
    )
  fi
fi

link_file() {
  local src="$1"
  local dst="$2"
  if [[ ! -e "$src" ]]; then
    error "$src: no such file or directory"
  elif [[ -e "$dst" ]]; then
    if [[ -z "$force" ]]; then
      error "file $dst exists"
    elif [[ -L "$dst" ]]; then
      warning "symlink $dst exists; removing..."
      checked rm "$dst"
    elif [[ -f "$dst" ]]; then
      warning "file $dst exists; removing..."
      checked rm "$dst"
    else
      error "$dst: not a symlink or file; not removing"
    fi
  fi
  checked ln -s "$src" "$dst"
}

if [[ -n "$pyinit" ]]; then
  if [[ ! -e "$pydir" ]]; then
    info "directory $pydir does not exist; creating..."
    checked mkdir -p "$pydir"
  fi
  info "linking pybullet assets to $pydir"
  link_file "$b3dir/$b3bdir/examples/pybullet/pybullet.so" "$pydir/pybullet.so"
  link_file "$b3dir/$b3bdir/examples/pybullet/pybullet_data" "$pydir/pybullet_data"
  link_file "$b3dir/$b3bdir/examples/pybullet/pybullet_envs" "$pydir/pybullet_envs"
  link_file "$b3dir/$b3bdir/examples/pybullet/pybullet_utils" "$pydir/pybullet_utils"
fi

if [[ -z "$PYTHONPATH" ]]; then
  warning "PYTHONPATH: environment not set"
  warning "Please read the PYTHONPATH section of the README for directions on fixing this problem"
fi

# prompt_any() {
#   local reply=
#   while true; do
#     read -p "$1: " reply
#     if [[ -z "$reply" ]]; then
#       echo "Please enter a response" >&2
#     else
#       break
#     fi
#   done
#   echo "$reply"
# }

# prompt_yes_no() {
#   local reply=
#   while true; do
#     read -p "$1: " reply
#     case "$reply" in
#       y|yes) break;;
#       n|no) break;;
#       *) echo "Please type either yes or no" >&2;;
#     esac
#   done
#   case "$reply" in
#     y*|Y*) return $T;;
#     n*|N*) return $F;;
#   esac
# }

# if prompt_yes_no "Fetch bullet3 from $GIT_URL?"; then
#   read -p "Please enter the path for bullet3 (default $BULLET_DIR): " b3dir
#   if [[ -z "$b3dir" ]]; then
#     b3dir="$BULLET_DIR"
#   fi
#   if [[ -d "$b3dir" ]]; then
#     error "Directory already exists; please remove it before continuing"
#   fi
# else
#   read -p "Please enter the path to bullet3: " b3dir
#   if [[ -d "$b3dir" ]] || [[ -L "$b3dir" ]]; then
#     echo "Found bullet3!" >&2
#   else
#     echo "That directory doesn't seem to exist"
#   fi
# fi
