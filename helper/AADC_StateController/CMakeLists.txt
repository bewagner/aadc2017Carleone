set(FILTER_NAME aadc_stateController)

   
adtf_add_filter(${FILTER_NAME}
            stdafx.h
            juryEnums.h
            cStateController.cpp            
            cStateController.h
            Maneuverlist.h
            ${HEADER_MOC}
)


# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} juryCommunication) 

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
    




