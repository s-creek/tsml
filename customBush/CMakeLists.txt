cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
project(hrpsys_choreonoid)

include(FindPkgConfig)

pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin choreonoid)
find_package(Boost REQUIRED system filesystem)
include_directories(${CHOREONOID_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(JVRC1Customizer SHARED src/JVRC1Customizer.cpp)
target_link_libraries(JVRC1Customizer ${CHOREONOID_LIBRARIES})
set_target_properties(JVRC1Customizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

foreach(cdir ${CHOREONOID_LIBRARY_DIRS})
  message(STATUS "${cdir}")
  if("${cdir}" MATCHES "choreonoid*")
    INSTALL(FILES JVRC1Customizer.so  DESTINATION  "${cdir}/customizer")
  endif()
endforeach()
#INSTALL(FILES JVRC1Customizer.so  DESTINATION  "${CHOREONOID_LIBRARY_DIRS}/customizer")

