# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_TheBrain)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    TheBrain.h
    TheBrain.cpp
)

adtf_set_folder(${FILTER_NAME} TheBrain) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
