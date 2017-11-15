# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_EmergencyStop)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    EmergencyStop.h
    EmergencyStop.cpp
)

adtf_set_folder(${FILTER_NAME} EmergencyStop) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
