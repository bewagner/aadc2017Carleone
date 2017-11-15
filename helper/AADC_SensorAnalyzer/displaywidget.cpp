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
* $Author:: spiesra $  $Date:: 2017-05-15 10:19:26#$ $Rev:: 63225   $
**********************************************************************/

#include "stdafx.h"
#include "displaywidget.h"
#include "cSensorAnalyzer.h"

DisplayWidget::DisplayWidget(QWidget* parent) :
    QWidget(parent)
{
// initialize the main widget
    m_pWidget = new QWidget(this);
    //m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    //m_pWidget->setFixedSize(500,500);

//initialize the fonts
    m_mainFont = new QFont("Arial",12);
    setFont(*m_mainFont);


// create title
    QLabel *titleLable = new QLabel("AADC Sensor Analyzer", this);
    titleLable->setFont(QFont("Arial",25));
    titleLable->setWordWrap(true);
    titleLable->setAlignment(Qt::AlignCenter);

// create the table view and the item model for the voltage sensors
    m_SensorItemModel = new QStandardItemModel(LIST_LENGTH,5,this);
    m_SensorTableView = new QTableView(this);
    m_SensorTableView->setModel(m_SensorItemModel);
    m_SensorTableView->setFont(QFont("Arial",8));
    m_SensorTableView->resizeRowsToContents();
    m_SensorTableView->verticalHeader()->setVisible(false);
    m_SensorTableView->setColumnWidth(0,150);
    m_SensorTableView->setColumnWidth(1,60);
    m_SensorTableView->setColumnWidth(2,60);
    m_SensorTableView->setColumnWidth(3,60);
    m_SensorTableView->setColumnWidth(4,60);
    m_SensorTableView->setFixedWidth(400);
    m_SensorTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_SensorTableView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_SensorTableView->setFixedHeight(m_SensorTableView->rowHeight(1)*(m_SensorItemModel->rowCount()+1));

    QStringList header;
    header << "sensor" << "Value"<< "Nominal" << "diff" << "Unit";
    m_SensorItemModel->setHorizontalHeaderLabels(header);
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_VOLTAGE_MEASUREMENT,0), "voltage measurement");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_VOLTAGE_MEASUREMENT,4), "mV");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_VOLTAGE_SPEEDCTR,0), "voltage speed contr");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_VOLTAGE_SPEEDCTR,4), "mV");

    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_LEFT,0), "ultrasonic front left");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_LEFT,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_CENTER_LEFT,0), "ultrasonic front center left");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_CENTER_LEFT,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_CENTER,0), "ultrasonic front center");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_CENTER,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_CENTER_RIGHT,0), "ultrasonic front center right");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_CENTER_RIGHT,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_RIGHT,0), "ultrasonic front right");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_FRONT_RIGHT,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_SIDE_LEFT,0), "ultrasonic side left");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_SIDE_LEFT,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_SIDE_RIGHT,0), "ultrasonic side right");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_SIDE_RIGHT,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_REAR_LEFT,0), "ultrasonic rear left");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_REAR_LEFT,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_REAR_CENTER,0), "ultrasonic rear center");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_REAR_CENTER,4), "cm");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_REAR_RIGHT,0), "ultrasonic rear right");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_ULTRASONIC_REAR_RIGHT,4), "cm");

    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_X_ACC,0), "Acc x-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_X_ACC,4), "g");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Y_ACC,0), "Acc y-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Y_ACC,4), "g");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Z_ACC,0), "Acc z-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Z_ACC,4), "g");

    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_X_ROT, 0), "Rot x-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_X_ROT, 4), "deg/sec");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Y_ROT, 0), "Rot y-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Y_ROT, 4), "deg/sec");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Z_ROT, 0), "rot z-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Z_ROT, 4), "deg/sec");

    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_X_MAG, 0), "Grav x-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_X_MAG, 4), "mT");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Y_MAG, 0), "Grav y-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Y_MAG, 4), "mT");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Z_MAG, 0), "Grav z-axis");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_Z_MAG, 4), "mT");

    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_YAW,0), "yaw");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_YAW,4), "deg");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_PITCH,0), "pitch");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_PITCH,4), "deg");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_ROLL,0), "roll");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_GYROSCOPE_ROLL,4), "deg");

    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_TACH_RIGHT,0), "wheel tach right");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_TACH_RIGHT,4), "");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_TACH_LEFT,0), "wheel tach left");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_TACH_LEFT,4), "");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_DIR_RIGHT,0), "wheel dir right");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_DIR_RIGHT,4), "");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_DIR_LEFT,0), "wheel left right");
    m_SensorItemModel->setData(m_SensorItemModel->index(LIST_WHEEL_DIR_LEFT,4), "");

//Dropdown menu for file choosing
    m_dropDownFileChooser = new QComboBox(this);

// creating the main layout
    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(titleLable,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_dropDownFileChooser, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_SensorTableView,0,Qt::AlignCenter);
    setLayout(m_mainLayout);
}

DisplayWidget::~DisplayWidget()
{
}

void DisplayWidget::SetDirectory(QString filename)
{
    QDir dir(filename);

    if(!dir.isAbsolute())
        LOG_WARNING(cString::Format("Path is not absolute. Force absolute path!"));

    QStringList fileList = dir.entryList(QDir::Files);
    QStringList files;

    for(QStringList::iterator it = fileList.begin(); it != fileList.end(); it++)
    {
        if(it->endsWith(".xml"))
        {
            cDOM oDOM;
            QString qString = dir.absoluteFilePath(".") + "/" + *it;
            LOG_INFO(cString::Format("path %s", qString.toStdString().c_str()));
            oDOM.Load(qString.toStdString().c_str());
            cDOMElementRefList oElems;
            if(IS_OK(oDOM.FindNodes("presets/sensorPreset", oElems)))
                files.push_back(*it);
        }
    }

    m_dropDownFileChooser->addItems(files);
}


void DisplayWidget::SetSensorData(int senorListId, float value)
{
    //plot the measurement value
    m_SensorItemModel->setData(m_SensorItemModel->index(senorListId,1),QString::number(value,'f',2));

    //mark cell colors red or green
    if ((value<(m_sensorPresets[senorListId].nominalValue-m_sensorPresets[senorListId].maxNegDeviation) ||  (value>(m_sensorPresets[senorListId].nominalValue+m_sensorPresets[senorListId].maxPosDeviation))))
    {
        m_SensorItemModel->setData(m_SensorItemModel->index(senorListId,1),QBrush(Qt::red),Qt::BackgroundColorRole);
        m_SensorItemModel->setData(m_SensorItemModel->index(senorListId,3),QBrush(Qt::red),Qt::BackgroundColorRole);
    }
    else
    {
        m_SensorItemModel->setData(m_SensorItemModel->index(senorListId,1),QBrush(Qt::green),Qt::BackgroundColorRole);
        m_SensorItemModel->setData(m_SensorItemModel->index(senorListId,3),QBrush(Qt::green),Qt::BackgroundColorRole);
    }

    //plot difference
    m_SensorItemModel->setData(m_SensorItemModel->index(senorListId,3),QString::number(value-m_sensorPresets[senorListId].nominalValue,'f',2));
}


void DisplayWidget::SetSensorPresets(vector<tSensorPreset> &sensorPresets)
{
    m_sensorPresets = sensorPresets;
    for (int i=0; i<static_cast<int>(m_sensorPresets.size()); i++)
    {
        m_SensorItemModel->setData(m_SensorItemModel->index(i,2), QString::number(m_sensorPresets[i].nominalValue,'f',2));
    }
}

