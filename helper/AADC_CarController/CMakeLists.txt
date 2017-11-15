set(FILTER_NAME aadc_CarController)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)
    # define header for qt moc
	set(HEADER_MOC
		widget.h
		cCarController.h
		)
				
	adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
                )
	
	adtf_add_ui(${FILTER_NAME}
                ui/ui_widget.ui
                )
	
	adtf_add_filter(${FILTER_NAME}
        stdafx.h
		widget.cpp
		cCarController.cpp
		${HEADER_MOC}
	)


    #target_link_libraries( ${FILTER_NAME} ${SFML_LIBRARIES} )
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} helper) 
    
	adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
	
endif(QT_FOUND)



