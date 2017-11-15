/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra  $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

#include "FrameCounter.h"
#include "aadc_structs.h"
#include "aadc_enums.h"

namespace Ui
{
/*! A widget. */
class Widget;
}


/*! the main  */
class Widget : public QWidget
{
    Q_OBJECT

public:

    /*!
     * Constructor.
     *
     * \param [in,out]  parent  (Optional) If non-null, the parent.
     */
    explicit Widget(QWidget *parent = 0);
    /*! Destructor. */
    ~Widget();

public slots:

    /*!
     * Sets us fl.
     *
     * \param   distance    The distance from ultrasonic sensor front left
     */
    void setUsFL(int distance);

    /*!
     * Sets us fcl.
     *
     * \param   distance    The distance from ultrasonic sensor front center left
     */
    void setUsFCL(int distance);

    /*!
     * Sets us fc.
     *
     * \param   distance    The distance from ultrasonic sensor front center
     */
    void setUsFC(int distance);

    /*!
     * Sets us fcr.
     *
     * \param   distance    The distance from ultrasonic sensor front center right
     */
    void setUsFCR(int distance);

    /*!
     * Sets us fr.
     *
     * \param   distance    The distance from ultrasonic sensor front right
     */
    void setUsFR(int distance);

    /*!
     * Sets us sl.
     *
     * \param   distance    The distance from ultrasonic sensor side left
     */
    void setUsSL(int distance);

    /*!
     * Sets us rcl.
     *
     * \param   distance    The distance from ultrasonic sensor rear center left
     */
    void setUsRCL(int distance);

    /*!
     * Sets us rectangle.
     *
     * \param   distance    The distance from ultrasonic sensor rear center
     */
    void setUsRC(int distance);

    /*!
     * Sets us rcr.
     *
     * \param   distance   The distance from ultrasonic sensor rear center right
     */
    void setUsRCR(int distance);

    /*!
     * Sets us sr.
     *
     * \param   distance    The distance from ultrasonic sensor side right
     */
    void setUsSR(int distance);

    /*!
     * Sets an imu.
     *
     * \param   ax      The ax.
     * \param   ay      The ay.
     * \param   az      The az.
     * \param   gx      The gx.
     * \param   gy      The gy.
     * \param   gz      The gz.
     * \param   mx      The mx.
     * \param   my      my.
     * \param   mz      The mz.
     * \param   roll    The roll.
     * \param   pitch   The pitch.
     * \param   yaw     The yaw.
     */
    void setImu(double ax, double ay, double az,
                double gx, double gy, double gz,
                double mx, double my, double mz,
                double roll, double pitch, double yaw);

    /*!
     * Sets wheel left.
     *
     * \param   count       Number of wheel
     * \param   direction   The direction of wheel
     */
    void setWheelLeft(int count, int direction);

    /*!
     * Sets wheel right.
     *
     * \param   count       counter of wheel
     * \param   direction   The direction of wheel.
     */
    void setWheelRight(int count, int direction);

    /*!
     * Sets voltage actuator.
     *
     * \param   value   The value.
     */
    void setVoltageActuator(float value);

    /*!
     * Sets voltage actuator cell 1.
     *
     * \param   value   The value.
     */
    void setVoltageActuatorCell1(float value);

    /*!
     * Sets voltage actuator cell 2.
     *
     * \param   value   The value.
     */
    void setVoltageActuatorCell2(float value);

    /*!
     * Sets voltage sensors.
     *
     * \param   value   The value.
     */
    void setVoltageSensors(float value);

    /*!
     * Sets voltage sensors cell 1.
     *
     * \param   value   The value.
     */
    void setVoltageSensorsCell1(float value);

    /*!
     * Sets voltage sensors cell 2.
     *
     * \param   value   The value.
     */
    void setVoltageSensorsCell2(float value);

    /*!
     * Sets voltage sensors cell 3.
     *
     * \param   value   The value.
     */
    void setVoltageSensorsCell3(float value);

    /*!
     * Sets voltage sensors cell 4.
     *
     * \param   value   The value.
     */
    void setVoltageSensorsCell4(float value);

    /*!
     * Sets voltage sensors cell 5.
     *
     * \param   value   The value.
     */
    void setVoltageSensorsCell5(float value);

    /*!
     * Sets voltage sensors cell 6.
     *
     * \param   value   The value.
     */
    void setVoltageSensorsCell6(float value);

    /*! Updates the graphical user interface. */
    void update_gui();

private:
    /*! The user interface */
    Ui::Widget *ui;

    /*! The ultrasonic data */
    tUltrasonicStructSimple m_usData;

    /*! the imu data */
    tInerMeasUnitData m_imuData;

    /*! A wheel data. */
    tWheelDataSimple m_wheelData;

    /*! holds the battery data */
    tVoltageStructSimple m_batteryData;

    /*! The frame rates[ size] */
    FrameCounter m_frameRates[SensorDefinition::sensorCount];
};

#endif // WIDGET_H
