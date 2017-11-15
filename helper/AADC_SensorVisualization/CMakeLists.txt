set(FILTER_NAME aadc_SensorVisualization)

adtf_use_qt(QtCore QtGui)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

if(QT_FOUND)
    set(HEADER_MOC
        widget.h
        uswidget.h
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
        uswidget.cpp
        cSensorVisualization.h
        cSensorVisualization.cpp
        FrameCounter.h
        FrameCounter.cpp
        ${HEADER_MOC}
    )

    # set folder (only needed and available for MSVC)    
    adtf_set_folder(${FILTER_NAME} helper) 

    TARGET_LINK_LIBRARIES( ${FILTER_NAME} ${Boost_LIBRARIES} ) 

    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

endif(QT_FOUND)



