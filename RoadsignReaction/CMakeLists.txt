# add the include directory of the filter
INCLUDE_DIRECTORIES(../include)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
# declare our filter
set(FILTER_NAME user_RoadsignReaction)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cRoadsignReaction.h
    cRoadsignReaction.cpp
)

adtf_set_folder(${FILTER_NAME} RoadsignReaction) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
