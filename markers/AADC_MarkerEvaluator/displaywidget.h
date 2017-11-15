/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef DISPWIDGET
#define DISPWIDGET


#include "stdafx.h"
#include "qcustomplot.h"

#include "aadc_roadSign_enums.h"

#define RANGE_DATASET 10

namespace Ui
{
class DisplayWidget;
}

class DisplayWidget : public QWidget
{
    Q_OBJECT

public:
    /*! Default Constructor
    * \param parent parent widget
    */
    DisplayWidget(QWidget* parent);

    /*! default destructor */
    virtual ~DisplayWidget();
    /*! this function pushes the data received by filter to the m_identiferBuffer
    *\param data of sign to push
    */
    void pushData(tInt16 data);

private slots:
    /*!setup all graphs for the road signs*/
    void setupGraph();
    /*! this function handles the realtime graph*/
    void realTimeData();
    /*! this function calculates the data in m_identiferBuffer*/
    void calcData();
    /*! this function sets the roadsign in the gui
    * \param sign integer id of road sign
    */
    void setRoadSign(int sign);

private:
    /*! get the corresponding index of the given sign
    * \param signId the id of the sign
    * \return index of sign in list
    */
    tInt16 getListIndexOfSign(tInt16 signId);

    /*!Pointer to the Gui*/
    Ui::DisplayWidget *ui;
    /*! this Timer is handles the update rate of the graph*/
    QTimer dataTimer;

    /*! graph to display values*/
    QCPGraph* m_graph_giveway;
    /*! graph to display values*/
    QCPGraph* m_graph_haveway;
    /*! graph to display values*/
    QCPGraph* m_graph_stopandgiveway;
    /*! graph to display values*/
    QCPGraph* m_graph_parkingarea;
    /*! graph to display values*/
    QCPGraph* m_graph_aheadonly;
    /*! graph to display values*/
    QCPGraph* m_graph_unmarkedintersection;
    /*! graph to display values*/
    QCPGraph* m_graph_pedestriancrossing;
    /*! graph to display values*/
    QCPGraph* m_graph_roundabout;
    /*! graph to display values*/
    QCPGraph* m_graph_noovertaking;
    /*! graph to display values*/
    QCPGraph* m_graph_noentry;
    /*! graph to display values*/
    QCPGraph* m_graph_oneway;
    /*! The graph testcourse a 9 */
    QCPGraph* m_graph_testcourseA9;
    /*! The graph roadworks */
    QCPGraph* m_graph_roadworks;
    /*! The graph kilometres per hour 50 */
    QCPGraph* m_graph_kmh50;
    /*! The graph kilometres per hour 100 */
    QCPGraph* m_graph_kmh100;

    /*! Array which holds the RoadSign Images*/
    QImage* m_roadSigns[roadsignIDs::NUM_ROADSIGNS];

    /*! Buffer to hold the values received by the filter*/
    std::deque<tInt16> m_identiferBuffer;
};

#endif
