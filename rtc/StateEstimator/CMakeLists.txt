cmake_minimum_required(VERSION 2.6)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if (NOT CMAKE_MODULE_PATH)
  set(CMAKE_MODULE_PATH $ENV{TSML_DIR}/share/cmake_modules/ CACHE STRING "set path to cmake_modules")
endif (NOT CMAKE_MODULE_PATH)
set(CMAKE_CXX_FLAGS -Wall)

####
## settings for OpenRTM
##
include(FindPkgConfig)
include(CheckIncludeFiles)
pkg_check_modules(OPENRTM REQUIRED openrtm-aist)

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
include_directories(${CMAKE_MODULE_PATH})
include(GenerateStubSkel)

###
# add matrix
#
if(NOT USE_CNOID_MODEL)
  add_definitions(-DUSE_HRP_MODEL)
else()
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  include_directories(${EIGEN3_INCLUDE_DIRS})
  add_definitions(-DUSE_CNOID_MODEL)
endif()

####
## for each component
##
set(comp_name ${COMP_NAME})
set(comp_sources ${comp_name}.cpp)

#generate_stub_skel(${comp_name}Service)
#set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)

set(comp_sources ${comp_sources} KalmanFilter.cpp)


add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES})
#target_link_libraries(${comp_name} lapack blas f2c m)
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES})
  target_link_libraries(${comp_name}Comp boost_filesystem)
  #target_link_libraries(${comp_name}Comp m)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")

#install(TARGETS ${target}
#install(TARGETS ${comp_name} LIBRARY DESTINATION rtc CONFIGURATIONS)
install(TARGETS ${comp_name} LIBRARY DESTINATION lib/rtc CONFIGURATIONS)
#install(FILES connect.conf DESTINATION rtc)
