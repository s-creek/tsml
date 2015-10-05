cmake_minimum_required(VERSION 2.6)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if (NOT CMAKE_MODULE_PATH)
  set(CMAKE_MODULE_PATH /home/player/tsml/share/hrpsys/cmake_modules/ CACHE STRING "set path to cmake_modules")
endif (NOT CMAKE_MODULE_PATH)
add_definitions(-std=gnu++0x)


####
## settings for OpenRTM
##
include(FindPkgConfig)
include(CheckIncludeFiles)
pkg_check_modules(OPENRTM REQUIRED openrtm-aist)

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

# for GenerateStubSkel
execute_process(
  COMMAND $ENV{RTM_ROOT}/bin/rtm-config --rtm-idldir
  OUTPUT_VARIABLE OPENRTM_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(GenerateStubSkel)


###
# setting choreonoid
#
pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})


####
# setting OpenCV
#
pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


####
# setting ZBar
#
pkg_check_modules(ZBAR REQUIRED zbar)
include_directories(${ZBAR_INCLUDE_DIRS})
link_directories(${ZBAR_LIBRARY_DIRS})


include_directories(${CMAKE_MODULE_PATH})
#include_directories(../../creeklib/creekQrCodeDetector)


####
## for each component
##
set(comp_name ${COMP_NAME})
set(comp_sources ${comp_name}.cpp)

generate_stub_skel(${comp_name}Service)
set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)

set(comp_sources ${comp_sources} CameraPort.cpp)
#set(comp_sources ${comp_sources} ../../creeklib/creekQrCodeDetector/creekQrCodeDetector.cpp)

add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES})
target_link_libraries(${comp_name} ${CHOREONOID_LIBRARIES} CnoidOpenRTM)
target_link_libraries(${comp_name} ${OPENCV_LIBRARIES})
target_link_libraries(${comp_name} ${ZBAR_LIBRARIES})
target_link_libraries(${comp_name} lapack blas f2c m)
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES})
  target_link_libraries(${comp_name}Comp ${CHOREONOID_LIBRARIES} CnoidOpenRTM)
  target_link_libraries(${comp_name}Comp ${OPENCV_LIBRARIES})
  target_link_libraries(${comp_name}Comp ${ZBAR_LIBRARIES})
  target_link_libraries(${comp_name}Comp boost_filesystem)
  target_link_libraries(${comp_name}Comp m)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")

#install(TARGETS ${target}
#install(TARGETS ${comp_name} LIBRARY DESTINATION rtc CONFIGURATIONS)
install(TARGETS ${comp_name} LIBRARY DESTINATION lib CONFIGURATIONS)
#install(FILES connect.conf DESTINATION rtc)
