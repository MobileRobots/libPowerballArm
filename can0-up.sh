#/bin/sh
set -x
set -e
ip link set can0 type can bitrate 500000
ip link set up can0
