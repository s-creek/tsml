#!/bin/sh

export PATH=/usr/bin:$PATH
export PKG_CONFIG_PATH=/usr/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH:/opt/grx/lib

cd `dirname $0`

ln -sfv rtc.conf rtc.conf.choreonoid
TASK_DIR=/home/player/tsml/model/tasks

#/usr/bin/choreonoid $@ #--start-simulation
#/usr/bin/choreonoid $@ 2>&1 | tee log.dat

/usr/bin/choreonoid ${TASK_DIR}/O1/O1.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/O2/O2.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R11L/R11L.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R11M/R11M.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R12/R12.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R2AB/R2AB.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R2C/R2C.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R3A/R3A.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R3B/R3B.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R4/R4.cnoid
#/usr/bin/choreonoid ${TASK_DIR}/R5/R5.cnoid