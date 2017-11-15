include_directories(../include)
set(FILTER_NAME StateControllerOptiCar)

   
adtf_add_filter(${FILTER_NAME}
            stdafx.h
            juryEnums.h
            cStateControllerOptiCar.cpp            
            cStateControllerOptiCar.h
            Maneuverlist.h
            ${HEADER_MOC}
)

adtf_set_folder(${FILTER_NAME} StateController) 
# set folder (only needed and available for MSVC)
#adtf_set_folder(${FILTER_NAME} aadcuser/stateControllerOptiCar) 

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
    




