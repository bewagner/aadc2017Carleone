/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-15 09:45:46#$ $Rev:: 63214   $
**********************************************************************/

#include "stdafx.h"

#ifndef DISPWIDGET
#define DISPWIDGET
#define LIST_LENGTH 28
#define LIST_VOLTAGE_MEASUREMENT 0
#define LIST_VOLTAGE_SPEEDCTR 1
#define LIST_ULTRASONIC_FRONT_LEFT 2
#define LIST_ULTRASONIC_FRONT_CENTER_LEFT 3
#define LIST_ULTRASONIC_FRONT_CENTER 4
#define LIST_ULTRASONIC_FRONT_CENTER_RIGHT 5
#define LIST_ULTRASONIC_FRONT_RIGHT 6
#define LIST_ULTRASONIC_SIDE_LEFT 7
#define LIST_ULTRASONIC_SIDE_RIGHT 8
#define LIST_ULTRASONIC_REAR_LEFT 9
#define LIST_ULTRASONIC_REAR_CENTER 10
#define LIST_ULTRASONIC_REAR_RIGHT 11
#define LIST_GYROSCOPE_X_ACC 12
#define LIST_GYROSCOPE_Y_ACC 13
#define LIST_GYROSCOPE_Z_ACC 14
#define LIST_GYROSCOPE_YAW 15
#define LIST_GYROSCOPE_PITCH 16
#define LIST_GYROSCOPE_ROLL 17
#define LIST_GYROSCOPE_X_ROT 18
#define LIST_GYROSCOPE_Y_ROT 19
#define LIST_GYROSCOPE_Z_ROT 20
#define LIST_GYROSCOPE_X_MAG 21
#define LIST_GYROSCOPE_Y_MAG 22
#define LIST_GYROSCOPE_Z_MAG 23
#define LIST_WHEEL_TACH_RIGHT 24
#define LIST_WHEEL_TACH_LEFT 25
#define LIST_WHEEL_DIR_RIGHT 26
#define LIST_WHEEL_DIR_LEFT 27

/*! struct for sensor presets */
struct tSensorPreset
{
    /*! name of sensor */
    cString sensorName;
    /*! nominal value for sensor */
    tFloat32 nominalValue;
    /*! the maximum allowed negative deviation */
    tFloat32 maxNegDeviation;
    /*! the maximum allowed positive deviation */
    tFloat32 maxPosDeviation;
    /*! default contstructor */
    tSensorPreset() :
        sensorName(""),
        nominalValue(0),
        maxNegDeviation(0),
        maxPosDeviation(0) {}
};


/*! coordinates from the car middle */
struct usSensorGeometrics
{
    /*! the x-position */
    tInt xPos;
    /*! the y-position */
    tInt yPos;
    /*! the start angle */
    tInt startAngle;
    /*! the maximum value */
    tFloat32 maxValue;
};

class cSensorAnalyzer;

namespace Ui
{
class DisplayGUI;
}

/*! this is the widget for the sensor anyalyzer*/
class DisplayWidget : public QWidget
{
    Q_OBJECT

public:
    /*! default constructor
    * \param parent the parent widget
    */
    DisplayWidget(QWidget* parent);

    /*! default constructor */
    ~DisplayWidget();

    /*!
    * \param sensorPresets a vector with the sensor presets
    */
    void SetSensorPresets(vector<tSensorPreset> &sensorPresets);

public slots:
    /*! slot for receiving data of ultrasonic sensors for gui
    * \param senorListId type of ultrasonic sensor
    * \param value value of ultrasonic data
    */
    void SetSensorData(int senorListId, float value);

    /*! slot for setting the dir path
    * \param pathname absolute path to the dir
    */
    void SetDirectory(QString pathname);

public:

    /*!
     * returns the widget pointer of the combo box.
     *
     * \return  dropDownWidget if analyzer
     */
    QComboBox* GetDropDownWidget()
    {
        return m_dropDownFileChooser;
    }

private:

    /*! the main widget */
    QWidget* m_pWidget;

    /*! the main font for the widget */
    QFont* m_mainFont;

    /*! the smaller main font for the tableviews etc */
    QFont* m_mainFontSmall;

    /*! the main layout of the widget */
    QVBoxLayout* m_mainLayout;

    /*! the dropdown filechooser */
    QComboBox* m_dropDownFileChooser;

    /*! the table view for the ultrasonic sensor data */
    QTableView* m_SensorTableView;

    /*! the item model for the ultrasonic sensor data */
    QStandardItemModel *m_SensorItemModel;

    /*! the presets loaded from xml file */
    vector<tSensorPreset> m_sensorPresets;
};

#endif
