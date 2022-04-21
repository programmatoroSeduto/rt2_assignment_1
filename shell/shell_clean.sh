#! /bin/bash

set -e

echo "deleting compiled bridge ... "
cd ../ws_bridge
./clean_bridge_ws.sh
echo "OK "

echo "deleting ros1 ws content ... "
cd ../ws_ros1
./clean_ros1_ws.sh
echo "OK"

echo "deleting ros2 ws content ... "
cd ../ws_ros2
./clean_ros2_ws.sh
echo "OK"

echo "deleting old logs ... "
cd ..
rm -rf logs
echo "OK"

cd shell
echo "Done."
