set(FILTER_NAME carStateGenerator)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)

    set(HEADER_MOC
        displaywidget.h
        CarStateGenerator.h
        )
    
    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )
    
    adtf_add_filter(${FILTER_NAME}
                    stdafx.h
                    displaywidget.cpp
                    CarStateGenerator.cpp
                    ${HEADER_MOC}
    )

    
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} helper) 
    
    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
    
endif(QT_FOUND)



