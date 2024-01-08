#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hua/shenlan-control/stanley/pro2_stanley/src/carla-ros-bridge/carla_walker_agent"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hua/shenlan-control/stanley/pro2_stanley/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hua/shenlan-control/stanley/pro2_stanley/install/lib/python3/dist-packages:/home/hua/shenlan-control/stanley/pro2_stanley/build/carla_walker_agent/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hua/shenlan-control/stanley/pro2_stanley/build/carla_walker_agent" \
    "/usr/bin/python3" \
    "/home/hua/shenlan-control/stanley/pro2_stanley/src/carla-ros-bridge/carla_walker_agent/setup.py" \
     \
    build --build-base "/home/hua/shenlan-control/stanley/pro2_stanley/build/carla_walker_agent" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hua/shenlan-control/stanley/pro2_stanley/install" --install-scripts="/home/hua/shenlan-control/stanley/pro2_stanley/install/bin"
