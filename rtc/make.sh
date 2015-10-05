#!/bin/sh
cd `dirname $0`
cur=`pwd`

list="ArmControlCartesian.cnoid PdServo ReferenceHolder.cnoid SequencePlayer.cnoid ST_cnoid StateEstimator sony_cnoid $@"

for dir in $list
do
    echo $cur
    cd ${cur}/${dir}
    ./make.sh
done