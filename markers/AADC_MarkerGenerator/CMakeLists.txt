cmake_minimum_required (VERSION 2.6)
set(MODULE_NAME aadc_markerGenerator)
project (${MODULE_NAME})

#set include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

add_executable(${MODULE_NAME} create_marker.cpp)

# linking libs
target_link_libraries(${MODULE_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
install (TARGETS ${MODULE_NAME} DESTINATION ${CMAKE_INSTALL_APPS})

set_property(TARGET ${MODULE_NAME} PROPERTY FOLDER marker)

if (WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS RelWithDebInfo Release)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_aruco${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_aruco${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS RelWithDebInfo Release)
    install(FILES "${Qt_DIR}/bin/QtGui4.dll" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS RelWithDebInfo Release Debug)
    install(FILES "${Qt_DIR}/bin/QtCore4.dll" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS RelWithDebInfo Release Debug)
    install(FILES "${Qt_DIR}/bin/QtTest4.dll" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS RelWithDebInfo Release Debug)
    install(FILES "create_markers.bat" DESTINATION ${CMAKE_INSTALL_APPS} CONFIGURATIONS RelWithDebInfo Release Debug)
endif (WIN32)