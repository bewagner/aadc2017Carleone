set(FILTER_NAME functions)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)

    set(HEADER_MOC
        functions.h
        )
    
    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )
    
    adtf_add_filter(${FILTER_NAME}
                    stdafx.h
                    functions.cpp
                    ${HEADER_MOC}
    )

    
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} helper) 
    
    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
    
endif(QT_FOUND)



