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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/bianjiang/catkin_ws/src/record_skeleton"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/bianjiang/catkin_ws/install/lib/python2.7/dist-packages:/home/bianjiang/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/bianjiang/catkin_ws/build" \
    "/usr/bin/python" \
    "/home/bianjiang/catkin_ws/src/record_skeleton/setup.py" \
    build --build-base "/home/bianjiang/catkin_ws/build/record_skeleton" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/bianjiang/catkin_ws/install" --install-scripts="/home/bianjiang/catkin_ws/install/bin"