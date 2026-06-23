#!/bin/bash

set -o errexit #exit on error

root=$(pwd)
build_dir=${root}/build/
# can_interface_rxy_dir=${root}/can_interface_rxy/
# UDP_Communication_rxy_dir=${root}/UDP_Communication_rxy/
# JoyStick_rxy_dir=${root}/JoyStick_rxy/
#build main
pushd ${build_dir}
sudo rm -r -f ./*
popd
# pushd ${can_interface_rxy_dir}
# sh autoclean.sh
# popd
# pushd ${UDP_Communication_rxy_dir}
# sh autoclean.sh
# popd
# pushd ${JoyStick_rxy_dir}
# sh autoclean.sh
# popd
