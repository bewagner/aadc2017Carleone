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
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QButtonGroup>
#include <QTimer>

namespace Ui
{
class Widget;
}

/*! this is the widget for the car controller filter */
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

    /*!
     * Gets light button group.
     *
     * \return  Null if it fails, else the light button group.
     */
    QButtonGroup* getLightButtonGroup();

    /*!
     * Gets ultrasonic button group.
     *
     * \return  Null if it fails, else the us button group.
     */
    QButtonGroup* getUsButtonGroup();

signals:

    /*!
     * Steering received.
     *
     * \param   value   The value.
     */
    void steeringReceived(float value);

    /*!
     * Throttle received.
     *
     * \param   value   The value.
     */
    void throttleReceived(float value);

    /*!
     * Button clicked.
     *
     * \param   button  The button clicked or not
     */
    void buttonClicked(int button);

public slots:

    /*!
     * Sets a speed value
     *
     * \param   value   The value.
     */
    void setSpeed(int value);

    /*!
     * Sets a steering value
     *
     * \param   value   The value.
     */
    void setSteering(int value);

private slots:

    /*!
     * Handles vertical slider throttle value changed signals.
     *
     * \param   value   The value.
     */
    void on_verticalSlider_throttle_valueChanged(int value);

    /*!
     * Handles horizontal slider steering value changed signals.
     *
     * \param   value   The value.
     */
    void on_horizontalSlider_steering_valueChanged(int value);

    /*! Handles push button calibrate clicked signals. */
    void on_pushButton_calibrate_clicked();

private:
    /*! The user interface */
    Ui::Widget *ui;
    /*! The timer */
    QTimer m_timer;

private slots:
    /*! Updates this object. */
    void update();
};

#endif // WIDGET_H
