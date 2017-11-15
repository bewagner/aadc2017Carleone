set(FILTER_NAME aadc_mapVisualization)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)


	include_directories(${OPENNI2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
	# define header for qt moc
    set(HEADER_MOC
        displaywidget.h
        cMapVisualization.h
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
        cMapVisualization.cpp
        ${HEADER_MOC}
    )

		target_link_libraries(${FILTER_NAME} ${OPENNI2_LIBS} ${OpenCV_LIBS})
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} position)

    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

endif(QT_FOUND)
