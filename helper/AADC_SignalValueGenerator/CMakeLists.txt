set(FILTER_NAME aadc_signalValueGenerator)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)

    set(HEADER_MOC
        displaywidget.h
        cSignalValueGenerator.h
        )
    
    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )
    
    adtf_add_filter(${FILTER_NAME}
                    stdafx.h
                    displaywidget.cpp
                    cSignalValueGenerator.h
                    cSignalValueGenerator.cpp
                    ${HEADER_MOC}
    )

    
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} helper) 
    
    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
    
endif(QT_FOUND)



