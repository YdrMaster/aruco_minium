# Install script for directory: /home/mechdancer/repos/aruco_demo/aruco/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1.15"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/mechdancer/repos/aruco_demo/aruco/build/src/libaruco.so.3.1.15"
    "/home/mechdancer/repos/aruco_demo/aruco/build/src/libaruco.so.3.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1.15"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES "/home/mechdancer/repos/aruco_demo/aruco/build/src/libaruco.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "/home/mechdancer/repos/aruco_demo/aruco/src/aruco_cvversioning.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/cameraparameters.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/dictionary_based.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/ippe.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/markerdetector_impl.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/markermap.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/timers.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/aruco_export.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/cvdrawingutils.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/dictionary.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/levmarq.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/marker.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/picoflann.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/aruco.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/debug.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/markerdetector.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/markerlabeler.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/posetracker.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/fractaldetector.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco/fractallabelers" TYPE FILE FILES
    "/home/mechdancer/repos/aruco_demo/aruco/src/fractallabelers/fractalposetracker.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/fractallabelers/fractalmarkerset.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/fractallabelers/fractalmarker.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/fractallabelers/fractallabeler.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco/dcf" TYPE FILE FILES
    "/home/mechdancer/repos/aruco_demo/aruco/src/dcf/dcfmarkermaptracker.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/dcf/dcfmarkertracker.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/dcf/dcf_utils.h"
    "/home/mechdancer/repos/aruco_demo/aruco/src/dcf/trackerimpl.h"
    )
endif()

