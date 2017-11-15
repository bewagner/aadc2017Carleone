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

#ifndef USWIDGET_H
#define USWIDGET_H

#include "stdafx.h"
#include "aadc_structs.h"

/*! the us widget for the visualization widget */
class uswidget : public QWidget
{
    Q_OBJECT
public:

    /*!
     * Constructor.
     *
     * \param [in,out]  parent  (Optional) parent widget.
     */
    explicit uswidget(QWidget *parent = 0);

signals:

public slots:
    /*! Animates this object. */
    void animate();

    /*!
     * Sets the distances.
     *
     * \param   us_front_left           The front left ultrasonic sensor data
     * \param   us_front_center_left    The front center left ultrasonic sensor data
     * \param   us_front_center         The front center ultrasonic sensor data
     * \param   us_front_center_right   The front center right ultrasonic sensor data
     * \param   us_front_right          The front right ultrasonic sensor data
     * \param   us_side_left            The side left ultrasonic sensor data
     * \param   us_side_right           The side right ultrasonic sensor data
     * \param   us_rear_center_left     The rear center left ultrasonic sensor data
     * \param   us_rear_center          The rear center ultrasonic sensor data
     * \param   us_rear_center_right    The rear center right ultrasonic sensor data
     */
    void setDistances(int us_front_left,
                      int us_front_center_left,
                      int us_front_center,
                      int us_front_center_right,
                      int us_front_right,
                      int us_side_left,
                      int us_side_right,
                      int us_rear_center_left,
                      int us_rear_center,
                      int us_rear_center_right);

protected:

    /*!
     * Paints the event described by event.
     *
     * \param [in,out]  event   If non-null, the event.
     */
    void paintEvent (QPaintEvent *event);

    /*!
     * Paint method
     *
     * \param [in,out]  painter If non-null, the painter.
     * \param [in,out]  event   If non-null, the event.
     */
    void paint(QPainter *painter, QPaintEvent *event);

    /*!
     * Draw filled arc.
     *
     * \param [in,out]  painter         If non-null, the painter.
     * \param           origin          The origin.
     * \param           startAngle      The start angle.
     * \param           spanAngle       The span angle.
     * \param           fillpercentage  The fillpercentage.
     */
    void drawFilledArc(QPainter *painter, QPointF origin, int startAngle, int spanAngle, int fillpercentage);

    /*!
     * Gets gradient color. [0, 100]
     *
     * \param   start   The start.
     * \param   end     The end.
     * \param   value   The value.
     *
     * \return  The gradient color.
     */
    QColor getGradientColor(QColor start, QColor end, int value);

private:
    /*! The elapsed */
    int m_elapsed;
    /*! Width of the canvas */
    int m_canvas_width;
    /*! Height of the canvas */
    int m_canvas_height;

    /*! The ultrasonic sensor data */
    tUltrasonicStructSimple m_usData;

    /*! The background */
    QBrush m_background;
    /*! The arcbrush */
    QBrush m_arcbrush;
    /*! The text font */
    QFont m_textFont;
    /*! The circle pen */
    QPen m_circlePen;
    /*! The text pen */
    QPen m_textPen;
};



#endif // USWIDGET_H
