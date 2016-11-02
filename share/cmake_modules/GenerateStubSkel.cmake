####
## settings for macro 
##
#set(idl_flags -bcxx -Wba -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -I${OPENRTM_DIR}/include/rtm/idl)
#set(idl_flags -bcxx -Wba -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -I${OPENRTM_IDL_DIR})
set(idl_flags -bcxx -Wba -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp )
set(package_path OpenHRP)
if(NOT QNXNTO)
  set(JAVAC javac)
  set(JAR jar)
  set(IDLJ idlj)
  set(idlj_flags -fclient -fserver -emitAll -td src -d ORBIT2_IDL -d TYPECODE_CORBA_PREFIX)
  set(javac_flags -target 1.5 -d . -sourcepath src)
endif()

####
## macro generate_stub_skel()
## 
macro(generate_stub_skel idl_basename)
  set(idl_file ${CMAKE_CURRENT_SOURCE_DIR}/${idl_basename}.idl)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  if(NOT QNXNTO)
    set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} ${CMAKE_CURRENT_BINARY_DIR}/python/${idl_basename}_idl.py
      COMMAND omniidl ${idl_flags} ${idl_file}
      COMMAND mkdir -p python
      #COMMAND omniidl -bpython -C python -I${OPENRTM_IDL_DIR} ${idl_file}
      #COMMAND ${IDLJ} ${idlj_flags} -I${OPENRTM_IDL_DIR} ${idl_file}
      COMMAND omniidl -bpython -C python ${idl_file}
      COMMAND ${IDLJ} ${idlj_flags} ${idl_file}
      COMMAND mkdir -p bin
      COMMAND ${JAVAC} ${javac_flags} src/*/*.java -d bin/
      COMMAND ${JAR} cf ${jarfile} -C ${CMAKE_CURRENT_BINARY_DIR}/bin .
      DEPENDS ${idl_file}
      )
    install(FILES ${idl_file} DESTINATION share/hrpsys/idl RENAME ${idl_basename}.idl${POSTFIX_FOR_ALTERNATIVES})
    install(FILES ${jarfile} DESTINATION share/hrpsys/jar RENAME ${idl_basename}.jar${POSTFIX_FOR_ALTERNATIVES})
    #install(FILES ${jarfile} DESTINATION share/java RENAME ${idl_basename}.jar${POSTFIX_FOR_ALTERNATIVES})
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/python/${idl_basename}_idl.py DESTINATION share/hrpsys/python RENAME ${idl_basename}_idl.py${POSTFIX_FOR_ALTERNATIVES})
    #install(FILES ${CMAKE_CURRENT_BINARY_DIR}/python/OpenHRP/__init__.py DESTINATION share/hrpsys/python/OpenHRP)
    #install(FILES ${CMAKE_CURRENT_BINARY_DIR}/python/OpenHRP__POA/__init__.py DESTINATION share/hrpsys/python/OpenHRP__POA)
  else()
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} 
      COMMAND omniidl ${idl_flags} ${idl_file}
      DEPENDS ${idl_file}
      )
  endif()
endmacro()

macro(generate_stub_skel_cpp idl_file)
  get_filename_component(dirName  ${idl_file} PATH)
  get_filename_component(baseName ${idl_file} NAME_WE)
  include_directories(${dirName})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${baseName}.hh ${CMAKE_CURRENT_SOURCE_DIR}/${baseName}Sk.cpp
    COMMAND omniidl ${idl_flags} ${idl_file}.idl
    DEPENDS ${idl_file}.idl
  )
endmacro()

##if (NOT DEFINED CMAKE_INSTALL_PREFIX)
#if (CMAKE_INSTALL_PREFIX MATCHES "/usr/local")
#  set(CMAKE_INSTALL_PREFIX "/opt/grx")
##endif(NOT DEFINED CMAKE_INSTALL_PREFIX)
#endif (CMAKE_INSTALL_PREFIX MATCHES "/usr/local")

#if (DEFINED ROBOT)
#set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/${ROBOT})
#endif (DEFINED ROBOT)

#if (NOT DEFINED IDL_DIR)
#  set(IDL_DIR /opt/grx/share/hrpsys/idl/) 
#endif(NOT DEFINED IDL_DIR)

####
## macro generate_stub_skel_wrapper()
##
## to introduce own data type for dataport
##
macro(generate_hrpsys_stub_skel_wrapper idl_basename)
  set(idl_file ${IDL_DIR}${idl_basename}.idl)
  set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar)
  if(NOT QNXNTO)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}SK.cc
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}DynSK.cc
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Stub.h
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Stub.cpp
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Skel.h
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Skel.cpp
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar
    COMMAND omniidl ${idl_flags} ${idl_file}
    COMMAND mv ${idl_basename}Sk.cpp ${idl_basename}SK.cc
    COMMAND cp ${idl_file} .
    COMMAND rtm-skelwrapper --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=${idl_basename}.idl
    COMMAND rm ${idl_basename}.idl
    COMMAND mkdir -p python
    COMMAND omniidl -bpython -C python -I${OPENRTM_IDL_DIR} ${idl_file}
    COMMAND ${IDLJ} ${idlj_flags} -I${OPENRTM_IDL_DIR} ${idl_file}
    COMMAND mkdir -p bin
    COMMAND ${JAVAC} ${javac_flags} src/*/*.java -d bin/
    COMMAND ${JAR} cf ${jarfile} -C ${CMAKE_CURRENT_BINARY_DIR}/bin .
    DEPENDS ${idl_file}
  )
  else()
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}SK.cc
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}DynSK.cc
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Stub.h
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Stub.cpp
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Skel.h
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Skel.cpp
    COMMAND omniidl ${idl_flags} ${idl_file}
    COMMAND mv ${idl_basename}Sk.cpp ${idl_basename}SK.cc
    COMMAND cp ${idl_file} .
    COMMAND rtm-skelwrapper --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=${idl_basename}.idl
    COMMAND rm ${idl_basename}.idl
    DEPENDS ${idl_file}
  )
  endif()
endmacro()

if(NOT CMAKE_BUILD_TYPE)
  set( CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()
