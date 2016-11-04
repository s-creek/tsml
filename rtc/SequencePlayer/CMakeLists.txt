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

# for GenerateStubSkel
execute_process(
  COMMAND $ENV{RTM_ROOT}/bin/rtm-config --rtm-idldir
  OUTPUT_VARIABLE OPENRTM_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(GenerateStubSkel)


###
# setting for OpenHRP or choreonoid
#
if(NOT USE_CNOID_MODEL)
  find_package(OpenHRP REQUIRED)
  include_directories(${OPENHRP_INCLUDE_DIRS} $ENV{HOME}/workspace/git)
  link_directories(${OPENHRP_LIBRARY_DIRS} $ENV{HOME}/workspace/git/creeklib/interpolator)
  set(MODEL_LIBRARIES ${OPENHRP_LIBRARIES} creekInterpolator_hrp)
  add_definitions(-DUSE_HRP_MODEL)
else()
  pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
  include_directories(${CHOREONOID_INCLUDE_DIRS} $ENV{HOME}/workspace/git)
  link_directories(${CHOREONOID_LIBRARY_DIRS} $ENV{HOME}/workspace/git/creeklib/interpolator)
  set(MODEL_LIBRARIES ${CHOREONOID_LIBRARIES} creekInterpolator_cnoid)

  add_definitions(-DUSE_CNOID_MODEL)
  string(REPLACE "." "" CNOID_VERSION ${CHOREONOID_VERSION})
  add_definitions(-DCNOID_VERSION=${CNOID_VERSION})
endif()


include_directories(${CMAKE_MODULE_PATH})


####
## for each component
##
set(comp_name ${COMP_NAME})
set(comp_sources ${comp_name}.cpp)

generate_stub_skel(${comp_name}Service)
set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)


add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES})
target_link_libraries(${comp_name} ${MODEL_LIBRARIES})
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES})
  target_link_libraries(${comp_name}Comp ${MODEL_LIBRARIES})
  target_link_libraries(${comp_name}Comp boost_filesystem)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")


install(TARGETS ${comp_name} LIBRARY DESTINATION rtc CONFIGURATIONS)
#install(FILES connect.conf DESTINATION rtc)
