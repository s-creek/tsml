cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_choreonoid)

include(FindPkgConfig)

pkg_check_modules(cnoid-plugin choreonoid-body-plugin)
find_package(Boost REQUIRED system filesystem)

if(${cnoid-plugin_FOUND})
  include_directories(${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  link_directories(${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} )
  add_library(JVRC1Customizer SHARED src/JVRC1Customizer.cpp)
  target_link_libraries(JVRC1Customizer ${cnoid-plugin_LIBRARIES})
  set_target_properties(JVRC1Customizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()

INSTALL(FILES JVRC1Customizer.so  DESTINATION  "/usr/lib/choreonoid-1.5/customizer")