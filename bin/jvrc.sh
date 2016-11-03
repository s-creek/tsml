#!/bin/sh

cd `dirname $0`

ln -sfv rtc.conf rtc.conf.choreonoid
TASK_DIR=/home/player/tsml/share/model/tasks
PROJECT_DIR=../share/project

#/usr/bin/choreonoid $@ #--start-simulation
#/usr/bin/choreonoid $@ 2>&1 | tee log.dat

/usr/bin/choreonoid ${PROJECT_DIR}/sample.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/O1/O1.cnoid

