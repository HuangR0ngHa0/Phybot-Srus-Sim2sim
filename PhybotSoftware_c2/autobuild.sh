#!/usr/bin/env bash
set -euo pipefail

sudo chmod 777 /dev/ttyUSB4

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_PATH1="$BASE_DIR/ThirdParty/urdfdom/lib"
LIB_PATH2="$BASE_DIR/ThirdParty/boost/lib"
EXPORT_LINE="export LD_LIBRARY_PATH=\"$LIB_PATH1:$LIB_PATH2:\$LD_LIBRARY_PATH\""

if grep -Fxq "$EXPORT_LINE" ~/.bashrc; then
    echo "LD_LIBRARY_PATH 已存在于 ~/.bashrc 中，无需重复添加"
else
    echo "$EXPORT_LINE" >> ~/.bashrc
    echo "已将 LD_LIBRARY_PATH 添加到 ~/.bashrc, 请重启！"
fi

source ~/.bashrc

echo "==================PHYBOT SOFTWARE=================="
echo "1.realrobot_mini"
echo "2.webots_sim"
echo "3.mujoco_sim_mini"
echo "4.gazebo_sim (only for catkin build   !!!!)"

read -p "enter control system number:" control_system_enter
case "$control_system_enter" in
    1)
        control_system_enter=realrobot_mini
        ;;
    2)
        control_system_enter=webots_sim
        ;;
    3)
        control_system_enter=mujoco_sim_mini
        ;;
    4)
        control_system_enter=gazebo_sim
        ;;
    *)
        echo "!invalid enter!"
        exit 1
        ;;
esac

root=$(pwd)
build_dir="$root/build"
mkdir -p "$build_dir"
echo "$build_dir"

if [ "$control_system_enter" = "mujoco_sim_mini" ]; then
    cmake -S "$root" -B "$build_dir" -DWHICH_ENV="$control_system_enter" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$build_dir" -- -j8
else
    pushd "$build_dir" >/dev/null
    cmake -DWHICH_ENV="${control_system_enter}" "$root" -DCMAKE_BUILD_TYPE=Release .
    make -j8
    popd >/dev/null
fi
