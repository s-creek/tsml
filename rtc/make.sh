#!/bin/sh
cd `dirname $0`
cur=`pwd`

. ./dirs.sh

list="$RTC_DIR_LIST $@"

for dir in $list
do
    echo $cur
    cd ${cur}/${dir}
    ./make.sh
done
