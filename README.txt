
Physics Simulation Using PyBullet

== DOWNLOADING AND INSTALLING BULLET ==

This simulation requires a custom build of bullet3. To download and build it,
please run the provided setup.sh script.

bash setup.sh [-h] [-c] [-d DIR] [-m DIR] [-b] [-p DIR] [-i] [-g ARGS] [-v] [-n] [-F]

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

The simplest "just set everything up, okay?" usage can be achieved via:

bash setup.sh -cbi

This downloads bullet3, builds it, and installs everything to $HOME/.python.

== CUSTOM BUILD ==

git clone https://github.com/Kaedenn/bullet3
cd bullet3
mkdir build_cmake && cd build_cmake
cmake ..
make -j4

== PYTHONPATH ==

If your $PYTHONPATH environment variable isn't set, then add the following to
the bottom of your ~/.bashrc, ~/.profile, ~/.kshrc, or the init file for
whichever shell you use:

export PYTHONPATH="$HOME/.python"

== INSTALLING ==

You need to deploy pybullet to a directory within your $PYTHONPATH, such as
$HOME/.python:

ln -s $PWD/examples/pybullet/pybullet.so $HOME/.python/pybullet.so
ln -s $PWD/examples/pybullet/pybullet_data $HOME/.python/pybullet_data
ln -s $PWD/examples/pybullet/pybullet_envs $HOME/.python/pybullet_envs
ln -s $PWD/examples/pybullet/pybullet_utils $HOME/.python/pybullet_utils

If $HOME/.python doesn't exist, then create it:

mkdir $HOME/.python

