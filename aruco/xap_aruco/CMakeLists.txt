INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/xap_aruco/xap_external_protocol)

if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()

add_subdirectory(xap_external_protocol)

add_executable(xap_aruco_gui xap_aruco_gui.cpp)
add_executable(xap_aruco xap_aruco.cpp)

target_link_libraries(xap_aruco_gui  XapExternalProtocol aruco opencv_calib3d opencv_highgui ${THREADLIB})
target_link_libraries(xap_aruco  XapExternalProtocol aruco opencv_calib3d opencv_highgui ${THREADLIB})

#add_executable(aruco_dummy aruco_dummy.cpp)
#target_link_libraries(aruco_dummy             aruco opencv_calib3d opencv_highgui ${THREADLIB})

#aruco_create_board
INSTALL(TARGETS xap_aruco_gui    RUNTIME DESTINATION bin)
