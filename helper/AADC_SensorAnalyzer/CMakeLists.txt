set(FILTER_NAME aadc_sensorAnalyzer)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)
    
	include_directories(
		"./../../../include"
	)
	
	# define header for qt moc
    set(HEADER_MOC
        displaywidget.h
        cSensorAnalyzer.h
        )
            
        
    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
                )
    
    adtf_add_qrc(${FILTER_NAME}
                #resource/car.qrc
                )
    
    adtf_add_ui(${FILTER_NAME}
                #ui/car.ui
                )
    
    adtf_add_filter(${FILTER_NAME}
        stdafx.h
        displaywidget.cpp
        cSensorAnalyzer.cpp
        ${HEADER_MOC}
    )
    
    
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} helper) 
    
    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
    
endif(QT_FOUND)



