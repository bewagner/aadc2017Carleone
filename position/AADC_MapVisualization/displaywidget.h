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
* $Author:: kuckal  $  $Date:: 2017-05-22 08:59:30#$ $Rev:: 63641   $
**********************************************************************/

#ifndef DISPWIDGET
#define DISPWIDGET

#define RESETCOUNTER 100
#define GRAPHICSSCENE_WIDTH 500
#define GRAPHICSSCENE_HEIGHT 500
#include "stdafx.h"

struct usSensorGeometrics{
// coordinates from the car middle
    tInt iXPos;
    tInt iYPos;
    tInt iStartAngle;
    tFloat32 f32MaxValue;
    };

class cMapVisualization;

namespace Ui {
class DisplayGUI;
}
/*!
 *     This is the display widget for the filter class of the visualizaion
 */
class DisplayWidget : public QWidget
{
    Q_OBJECT

    public:
        DisplayWidget(QWidget* parent);
        ~DisplayWidget();
    public slots:


    private:

        //Position Counter for View Reset
        tInt m_counter;

        /*! the main widget */
        QWidget* m_pWidget;

        /*! the main font for the widget */
        QFont* m_mainFont;

        /*! the smaller main font for the tableviews etc */
        QFont* m_mainFontSmall;

        /*! the x coordinate of the car in the graphicsscene */
        const qreal m_qPCarCenter_x;

        /*! the y coordinate of the car in the graphicsscene */
        const qreal m_qPCarCenter_y;

        QGraphicsScene* scene;
        /*! the main layout for the widget*/
        QVBoxLayout *m_mainLayout;

        //Graphics View for widget
        QGraphicsView* view;

        //Circles for Position
        QGraphicsEllipseItem *pos,*pos1;

        //Circles for Marker
        QGraphicsEllipseItem *marker1,*marker2;

        //Heading Line
        QGraphicsLineItem *head;

        //Text for Marker
        QGraphicsTextItem *text;

        //Rectangle for Parking
        QGraphicsRectItem *park;

        //Circle for Obstacle
        QGraphicsEllipseItem *obs;

        //Scales for Visuals
        float m_scalex,m_scaley,m_laneWidth,m_xmax,m_ymax;

        //initialize all Items
        void setItems();

        protected slots:
        //Render Map line by line
        void OnMapData(float x1, float y1, float x2, float y2);

        //Get Map scales - Original Map scale to visual scale
        void OnMapScale(float x1, float y1,float x2, float y2,float laneWidth);

        //Get Position and Render
        void OnSendPositionData(float x, float y,float h);

        //Get Marker and Render
        void OnSendMarkerData(float x,float y,int id);

        //Get Parking slot and Render
        void OnSendParkingData(int id,float x,float y,int status);

        //Get Obstacle and Render
        void OnSendObstacleData(float x,float y);

};

#endif
