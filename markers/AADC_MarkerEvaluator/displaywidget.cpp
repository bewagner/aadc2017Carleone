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

#include "stdafx.h"

#include "cMarkerEvaluator.h"
#include "displaywidget.h"
#include "display.h"

using namespace roadsignIDs;

DisplayWidget::DisplayWidget(QWidget* parent) : QWidget(parent), ui(new Ui::DisplayWidget)
{
    ui->setupUi(this);

    //load all image files from resource.qrc
    for(int i = 0; i < NUM_ROADSIGNS; i++)
        m_roadSigns[i] = new QImage();
    m_roadSigns[getListIndexOfSign(MARKER_ID_NOMATCH)]->load(":/resource/NoMatch.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_GIVEWAY)]->load(":/resource/Giveway.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_STOPANDGIVEWAY)]->load(":/resource/Stopandgiveway.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_HAVEWAY)]->load(":/resource/Haveway.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_AHEADONLY)]->load(":/resource/Aheadonly.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_PARKINGAREA)]->load(":/resource/Parkingarea.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_UNMARKEDINTERSECTION)]->load(":/resource/Unmarkedintersection.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_PEDESTRIANCROSSING)]->load(":/resource/Pedestriancrossing.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_ROUNDABOUT)]->load(":/resource/Roundabout.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_NOOVERTAKING)]->load(":/resource/Noovertaking.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_NOENTRYVEHICULARTRAFFIC)]->load(":/resource/Noentry.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_ONEWAYSTREET)]->load(":/resource/Oneway.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_TESTCOURSEA9)]->load(":/resource/TestcourseA9.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_ROADWORKS)]->load(":/resource/Roadworks.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_KMH50)]->load(":/resource/50kmh.png");
    m_roadSigns[getListIndexOfSign(MARKER_ID_KMH100)]->load(":/resource/100kmh.png");
    //add one graph
    m_graph_giveway = ui->customPlot->addGraph();
    m_graph_haveway = ui->customPlot->addGraph();
    m_graph_stopandgiveway = ui->customPlot->addGraph();
    m_graph_parkingarea = ui->customPlot->addGraph();
    m_graph_aheadonly = ui->customPlot->addGraph();
    m_graph_unmarkedintersection = ui->customPlot->addGraph();
    m_graph_pedestriancrossing= ui->customPlot->addGraph();
    m_graph_roundabout= ui->customPlot->addGraph();
    m_graph_noovertaking = ui->customPlot->addGraph();
    m_graph_noentry= ui->customPlot->addGraph();
    m_graph_oneway= ui->customPlot->addGraph();
    m_graph_testcourseA9 = ui->customPlot->addGraph();
    m_graph_roadworks = ui->customPlot->addGraph();
    m_graph_kmh50 = ui->customPlot->addGraph();
    m_graph_kmh100 = ui->customPlot->addGraph();
    //set sizepolic
    ui->image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->image->setScaledContents(true);

    setupGraph();
    setRoadSign(getListIndexOfSign(MARKER_ID_NOMATCH));
}

DisplayWidget::~DisplayWidget()
{
    for(int i = 0; i < NUM_ROADSIGNS; i++)
        delete m_roadSigns[i];
    delete ui;
}

void DisplayWidget::setRoadSign(int sign)
{
    ui->image->setPixmap(QPixmap::fromImage(*m_roadSigns[sign]));
}

void DisplayWidget::pushData(tInt16 data)
{
    m_identiferBuffer.push_back(data);
}


void DisplayWidget::setupGraph()
{
    ui->customPlot->legend->setVisible(true);

    //graph vorfahrtgew
    QPen pen_giveway;
    pen_giveway.setColor(Qt::red);
    m_graph_giveway->setPen(pen_giveway);
    m_graph_giveway->setName("Give Way");
    QPen pen_haveway;
    pen_haveway.setColor(Qt::darkGray);
    m_graph_haveway->setPen(pen_haveway);
    m_graph_haveway->setName("Have Way");
    QPen pen_stop;
    pen_stop.setColor(Qt::green);
    m_graph_stopandgiveway->setPen(pen_stop);
    m_graph_stopandgiveway->setName("Stop");
    QPen pen_parkingarea;
    pen_parkingarea.setColor(Qt::black);
    m_graph_parkingarea->setPen(pen_parkingarea);
    m_graph_parkingarea->setName("Parking Area");
    QPen pen_aheadonly;
    pen_aheadonly.setColor(Qt::blue);
    m_graph_aheadonly->setPen(pen_aheadonly);
    m_graph_aheadonly->setName("Ahead Only");
    QPen pen_unmarkedintersection;
    pen_unmarkedintersection.setColor(Qt::cyan);
    m_graph_unmarkedintersection->setPen(pen_unmarkedintersection);
    m_graph_unmarkedintersection->setName("Intersection");
    QPen pen_pedestriancrossing;
    pen_pedestriancrossing.setColor(Qt::magenta);
    m_graph_pedestriancrossing->setPen(pen_pedestriancrossing);
    m_graph_pedestriancrossing->setName("Pedestrian");
    QPen pen_roundabout;
    pen_roundabout.setColor(Qt::darkGreen);
    m_graph_roundabout->setPen(pen_roundabout);
    m_graph_roundabout->setName("Round About");
    QPen pen_noovertaking;
    pen_noovertaking.setColor(Qt::darkBlue);
    m_graph_noovertaking->setPen(pen_noovertaking);
    m_graph_noovertaking->setName("No Overtaking");
    QPen pen_noentry;
    pen_noentry.setColor(Qt::darkYellow);
    m_graph_noentry->setPen(pen_noentry);
    m_graph_noentry->setName("No Entry");
    QPen pen_oneway;
    pen_oneway.setColor(Qt::lightGray);
    m_graph_oneway->setPen(pen_oneway);
    m_graph_oneway->setName("One Way Street");
    QPen pen_testCourseA9;
    pen_testCourseA9.setColor(Qt::darkYellow);
    m_graph_testcourseA9->setPen(pen_testCourseA9);
    m_graph_testcourseA9->setName("Test course A9");
    QPen pen_Roadworks;
    pen_Roadworks.setColor(Qt::lightGray);
    m_graph_roadworks->setPen(pen_Roadworks);
    m_graph_roadworks->setName("Roadworks");
    QPen pen_kmh50;
    pen_kmh50.setColor(Qt::darkYellow);
    m_graph_kmh50->setPen(pen_kmh50);
    m_graph_kmh50->setName("50 kmh");
    QPen pen_kmh100;
    pen_kmh100.setColor(Qt::lightGray);
    m_graph_kmh100->setPen(pen_kmh100);
    m_graph_kmh100->setName("100 kmh");

    m_graph_giveway->setAntialiasedFill(false);
    m_graph_haveway->setAntialiasedFill(false);
    m_graph_stopandgiveway->setAntialiasedFill(false);
    m_graph_parkingarea->setAntialiasedFill(false);
    m_graph_aheadonly->setAntialiasedFill(false);
    m_graph_unmarkedintersection->setAntialiasedFill(false);
    m_graph_pedestriancrossing->setAntialiasedFill(false);
    m_graph_roundabout->setAntialiasedFill(false);
    m_graph_noovertaking->setAntialiasedFill(false);
    m_graph_noentry->setAntialiasedFill(false);
    m_graph_oneway->setAntialiasedFill(false);
    m_graph_testcourseA9->setAntialiasedFill(false);
    m_graph_roadworks->setAntialiasedFill(false);
    m_graph_kmh50->setAntialiasedFill(false);
    m_graph_kmh100->setAntialiasedFill(false);

    ui->customPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->customPlot->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->customPlot->xAxis->setAutoTickStep(false);
    ui->customPlot->xAxis->setTickStep(2);
    ui->customPlot->yAxis->setRange(-1, RANGE_DATASET+2);

    // setup a timer that repeatedly calls realtimeData:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realTimeData()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void DisplayWidget::realTimeData()
{
    double time = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    //as fast as possible calc the data
    calcData();

    // remove data of lines that's outside visible range:
    m_graph_giveway->removeDataBefore(time - 12);
    m_graph_haveway->removeDataBefore(time - 12);
    m_graph_stopandgiveway->removeDataBefore(time - 12);
    m_graph_parkingarea->removeDataBefore(time - 12);
    m_graph_aheadonly->removeDataBefore(time - 12);
    m_graph_unmarkedintersection->removeDataBefore(time - 12);
    m_graph_pedestriancrossing->removeDataBefore(time - 12);;
    m_graph_roundabout->removeDataBefore(time - 12);
    m_graph_noovertaking->removeDataBefore(time - 12);
    m_graph_noentry->removeDataBefore(time - 12);
    m_graph_oneway->removeDataBefore(time - 12);
    m_graph_testcourseA9->removeDataBefore(time - 12);
    m_graph_roadworks->removeDataBefore(time - 12);
    m_graph_kmh50->removeDataBefore(time - 12);
    m_graph_kmh100->removeDataBefore(time - 12);
    // make key axis range scroll with the data (at a constant range size of 8):
    ui->customPlot->xAxis->setRange(time + 0.25, 8, Qt::AlignRight);
    ui->customPlot->replot();
}

void DisplayWidget::calcData()
{
    tInt16 data;
    quint64 time = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    short int num_list[NUM_ROADSIGNS];
    for (int i = 0; i < NUM_ROADSIGNS; i++)
    {
        num_list[i] = 0;
    }

    if(m_identiferBuffer.size() > RANGE_DATASET)
    {
        for(int i = 0; i < RANGE_DATASET; i++)
        {
            data = m_identiferBuffer.front();
            m_identiferBuffer.pop_front();

            num_list[getListIndexOfSign(data)]++;
        }

        int tmp = 0, tmpIndex = 0;
        //get the max value
        for (int i = 0; i < NUM_ROADSIGNS; i++)
        {
            if(num_list[i] > tmp)
            {
                tmp = num_list[i];
                tmpIndex = i;
            }
        }

        setRoadSign(tmpIndex);

        m_graph_giveway->addData(time, num_list[getListIndexOfSign(MARKER_ID_GIVEWAY)]);
        m_graph_haveway->addData(time, num_list[getListIndexOfSign(MARKER_ID_HAVEWAY)]);
        m_graph_stopandgiveway->addData(time, num_list[getListIndexOfSign(MARKER_ID_STOPANDGIVEWAY)]);
        m_graph_parkingarea->addData(time, num_list[getListIndexOfSign(MARKER_ID_PARKINGAREA)]);
        m_graph_aheadonly->addData(time, num_list[getListIndexOfSign(MARKER_ID_AHEADONLY)]);
        m_graph_unmarkedintersection->addData(time, num_list[getListIndexOfSign(MARKER_ID_UNMARKEDINTERSECTION)]);
        m_graph_pedestriancrossing->addData(time, num_list[getListIndexOfSign(MARKER_ID_PEDESTRIANCROSSING)]);
        m_graph_roundabout->addData(time, num_list[getListIndexOfSign(MARKER_ID_ROUNDABOUT)]);
        m_graph_noovertaking->addData(time, num_list[getListIndexOfSign(MARKER_ID_NOOVERTAKING)]);
        m_graph_noentry->addData(time, num_list[getListIndexOfSign(MARKER_ID_NOENTRYVEHICULARTRAFFIC)]);
        m_graph_oneway->addData(time, num_list[getListIndexOfSign(MARKER_ID_ONEWAYSTREET)]);
        m_graph_testcourseA9->addData(time, num_list[getListIndexOfSign(MARKER_ID_TESTCOURSEA9)]);
        m_graph_roadworks->addData(time, num_list[getListIndexOfSign(MARKER_ID_ROADWORKS)]);
        m_graph_kmh50->addData(time, num_list[getListIndexOfSign(MARKER_ID_KMH50)]);
        m_graph_kmh100->addData(time, num_list[getListIndexOfSign(MARKER_ID_KMH100)]);
    }
}


tInt16 DisplayWidget::getListIndexOfSign(tInt16 signId)
{
    switch(signId)
    {
    case MARKER_ID_NOMATCH:
        return 0;
        break;
    case MARKER_ID_GIVEWAY:
        return 1;
        break;
    case MARKER_ID_HAVEWAY:
        return 2;
        break;
    case MARKER_ID_STOPANDGIVEWAY:
        return 3;
        break;
    case MARKER_ID_PARKINGAREA:
        return 4;
        break;
    case MARKER_ID_AHEADONLY:
        return 5;
        break;
    case MARKER_ID_UNMARKEDINTERSECTION:
        return 6;
        break;
    case MARKER_ID_PEDESTRIANCROSSING:
        return 7;
        break;
    case MARKER_ID_ROUNDABOUT:
        return 8;
        break;
    case MARKER_ID_NOOVERTAKING:
        return 9;
        break;
    case MARKER_ID_NOENTRYVEHICULARTRAFFIC:
        return 10;
        break;
    case MARKER_ID_TESTCOURSEA9:
        return 11;
        break;
    case MARKER_ID_ONEWAYSTREET:
        return 12;
        break;
    case MARKER_ID_ROADWORKS:
        return 13;
        break;
    case MARKER_ID_KMH50:
        return 14;
        break;
    case MARKER_ID_KMH100:
        return 15;
        break;
    default:
        return 0;
        break;
    }
}