# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
#set project name at beginning this is cleaner than just a folder
project(aadc_ExtPythonServer)
set(FILTER_NAME ${PROJECT_NAME})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR} 
    ${Thrift_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ../../libraries/ThriftExtIf/gen-ExtIf
	)

  set(HEADER
	stdafx.h 
 	ExtPythonServer.h
	)

  set(SOURCE
	ExtPythonServer.cpp
	)

  adtf_add_filter(${FILTER_NAME}
	${HEADER}
	${SOURCE}  
    
	)

  target_link_libraries(${FILTER_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${Thrift_LINK_LIBS}
    ${OpenSSL_LINK_LIBS}
    ThriftExtIf
	)

  adtf_set_folder(${FILTER_NAME} helper)

  adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

  if (WIN32)
	install(FILES "${BOOST_ROOT}/bin/boost_system-vc100-mt-1_58.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
	install(FILES "${BOOST_ROOT}/bin/boost_thread-vc100-mt-1_58.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
	install(FILES "${BOOST_ROOT}/bin/boost_chrono-vc100-mt-1_58.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
	install(FILES "${OpenSSL_DIR}/bin64/libeay32MD.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
	install(FILES "${OpenSSL_DIR}/bin64/ssleay32MD.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
	install(FILES "${OpenSSL_DIR}/bin64/libeay32MDd.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
	install(FILES "${OpenSSL_DIR}/bin64/ssleay32MDd.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
  endif (WIN32)
