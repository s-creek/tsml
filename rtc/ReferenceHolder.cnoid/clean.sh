#!/bin/sh
cd `dirname $0`

if [ -e Makefile ]; then
    make clean
fi


rm -f *~
rm -f *py.class


if [ "$1" = "-all" ]; then
    echo 'exe clean all'
    rm -f CMakeCache.txt
    rm -f Makefile
    rm -f cmake_install.cmake
    rm -f install_manifest.txt
    
    rm -rf CMakeFiles bin python src
fi