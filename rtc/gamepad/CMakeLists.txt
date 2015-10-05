cmake_minimum_required(VERSION 2.6)

set(IDL_DIR ${CMAKE_CURRENT_BINARY_DIR}/)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

####
## settings for OpenRTM and OpenHRP
##
include(FindPkgConfig)
include(CheckIncludeFiles)

pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

pkg_check_modules(CHOREONOID choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})


####
## for each component
##
set(comp_name GamepadRTC)
set(comp_sources ${comp_name}.cpp)

add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${CHOREONOID_LIBRARIES})
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES})

target_link_libraries(${comp_name} lapack blas f2c m)
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES} )
  target_link_libraries(${comp_name}Comp ${CHOREONOID_LIBRARIES})
  target_link_libraries(${comp_name}Comp boost_filesystem)
  target_link_libraries(${comp_name}Comp m)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")

#install(TARGETS ${comp_name} LIBRARY DESTINATION rtc CONFIGURATIONS)
install(TARGETS ${comp_name} LIBRARY DESTINATION lib CONFIGURATIONS)