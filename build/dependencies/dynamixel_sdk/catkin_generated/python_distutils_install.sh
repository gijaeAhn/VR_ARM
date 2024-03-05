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

echo_and_run cd "/home/sj/Desktop/VR_ARM/src/dependencies/dynamixel_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sj/Desktop/VR_ARM/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sj/Desktop/VR_ARM/install/lib/python3/dist-packages:/home/sj/Desktop/VR_ARM/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sj/Desktop/VR_ARM/build" \
    "/home/sj/anaconda3/bin/python3" \
    "/home/sj/Desktop/VR_ARM/src/dependencies/dynamixel_sdk/setup.py" \
     \
    build --build-base "/home/sj/Desktop/VR_ARM/build/dependencies/dynamixel_sdk" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sj/Desktop/VR_ARM/install" --install-scripts="/home/sj/Desktop/VR_ARM/install/bin"
