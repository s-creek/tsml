#!/bin/sh
cd `dirname $0`

#
# user configuration
#
export COMP_NAME="creekSequencePlayer"


#
# not change !!
#
export RTM_ROOT=/usr

#CMAKE_OPT="-DCMAKE_INSTALL_PREFIX:STRING=/usr/lib/choreonoid-1.5 -DCOMP_NAME=${COMP_NAME} -DUSE_CNOID_MODEL=ON"
CMAKE_OPT="-DCMAKE_INSTALL_PREFIX:STRING=/opt/tsml/lib -DCOMP_NAME=${COMP_NAME} -DUSE_CNOID_MODEL=ON"
MAKE_OPT="VERBOSE=1 $@"

# for logging
DATE=`/bin/date '+%Y%m%d%H%M'`
mkdir -p log

# cmake
cmake . ${CMAKE_OPT} 2>&1 | tee log/build_${DATE}.log

# make
make ${MAKE_OPT} 2>&1 | tee -a log/build_${DATE}.log

cd log
ln -sfv build_${DATE}.log build.log
