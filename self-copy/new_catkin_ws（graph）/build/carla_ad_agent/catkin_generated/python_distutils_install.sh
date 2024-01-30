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

echo_and_run cd "/home/hua/文档/new_catkin_ws（graph）/src/carla-ros-bridge/carla_ad_agent"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hua/文档/new_catkin_ws（graph）/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hua/文档/new_catkin_ws（graph）/install/lib/python3/dist-packages:/home/hua/文档/new_catkin_ws（graph）/build/carla_ad_agent/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hua/文档/new_catkin_ws（graph）/build/carla_ad_agent" \
    "/usr/bin/python3" \
    "/home/hua/文档/new_catkin_ws（graph）/src/carla-ros-bridge/carla_ad_agent/setup.py" \
     \
    build --build-base "/home/hua/文档/new_catkin_ws（graph）/build/carla_ad_agent" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hua/文档/new_catkin_ws（graph）/install" --install-scripts="/home/hua/文档/new_catkin_ws（graph）/install/bin"
