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

echo_and_run cd "/home/hua/shenlan-control/stanley/catkin_ws_correct/src/carla-ros-bridge/carla_ros_bridge"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hua/shenlan-control/stanley/catkin_ws_correct/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hua/shenlan-control/stanley/catkin_ws_correct/install/lib/python3/dist-packages:/home/hua/shenlan-control/stanley/catkin_ws_correct/build/carla_ros_bridge/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hua/shenlan-control/stanley/catkin_ws_correct/build/carla_ros_bridge" \
    "/usr/bin/python3" \
    "/home/hua/shenlan-control/stanley/catkin_ws_correct/src/carla-ros-bridge/carla_ros_bridge/setup.py" \
     \
    build --build-base "/home/hua/shenlan-control/stanley/catkin_ws_correct/build/carla_ros_bridge" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hua/shenlan-control/stanley/catkin_ws_correct/install" --install-scripts="/home/hua/shenlan-control/stanley/catkin_ws_correct/install/bin"
