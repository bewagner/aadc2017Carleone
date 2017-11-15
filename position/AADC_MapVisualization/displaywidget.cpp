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

#include "stdafx.h"
#include "displaywidget.h"
#include "cMapVisualization.h"
DisplayWidget::DisplayWidget(QWidget* parent) :
QWidget(parent), m_qPCarCenter_x(GRAPHICSSCENE_WIDTH), m_qPCarCenter_y(GRAPHICSSCENE_HEIGHT)
{
  //initialize Counter
  m_counter=0;
  //initialize LANEWIDTH
  m_laneWidth=1;
  // initialize the main widget
  m_pWidget = new QWidget(this);
  m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  m_pWidget->setFixedSize(GRAPHICSSCENE_WIDTH,GRAPHICSSCENE_HEIGHT);

  //initialize the fonts
  m_mainFont = new QFont("Arial",12);
  //Set Font
  m_mainFontSmall = new QFont("Arial",10);
  setFont(*m_mainFont);
  // create new QGraphicsScene
  scene=new QGraphicsScene(this);
  //Set Backgroud Coloe
  scene->setBackgroundBrush(QColor(43,43,43));
  //Create New view
  view= new QGraphicsView(scene,this);

  //initialize items
  setItems();
  //Add view layout
  view->setMaximumSize(600,800);
  //Map scent to view
  view->mapToScene(0,0);
  //Create View
  m_mainLayout = new QVBoxLayout();
  //Add view to the layout
  m_mainLayout->addWidget(view);
  setLayout(m_mainLayout);
}

DisplayWidget::~DisplayWidget()
{

}

//Get Map Scales and Store Values
void DisplayWidget::OnMapScale(float x1, float y1,float x2, float y2,float laneWidth)
{
  //Maximum x-coordinate
  m_xmax=x1;
  //Maximum y-coordinate
  m_ymax=y1;
  //Scale in x
  m_scalex=x2;
  //Scale in y
  m_scaley=y2;
  //Lane width in Metres
  m_laneWidth=laneWidth;
}


//Plot Map points
void DisplayWidget::OnMapData(float x1, float y1, float x2, float y2)
{
  //Project lane left and Right
  float projLength=m_laneWidth;

  //Heading of Lane Points
  float h1=atan2(y2-y1,x2-x1);
  //Create Right Lane Border Points
  float x11=x1+cos(h1+M_PI/2)*projLength*m_scalex;
  float y11=y1+sin(h1+M_PI/2)*projLength*m_scaley;
  float x21=x2+cos(h1+M_PI/2)*projLength*m_scalex;
  float y21=y2+sin(h1+M_PI/2)*projLength*m_scaley;

  //Create Left Lane Border Points
  float x12=x1+cos(h1-M_PI/2)*projLength*m_scalex;
  float y12=y1+sin(h1-M_PI/2)*projLength*m_scaley;
  float x22=x2+cos(h1-M_PI/2)*projLength*m_scalex;
  float y22=y2+sin(h1-M_PI/2)*projLength*m_scaley;

  //Create Right Lane
  QGraphicsLineItem *line1=scene->addLine(x11,y11,x21,y21);
  QPen pen1;
  pen1.setWidth(1);
  pen1.setColor(QColor(18,232,175));
  line1->setPen(pen1);

  //Create Left Line
  QGraphicsLineItem *line2=scene->addLine(x12,y12,x22,y22);
  QPen pen2;
  pen2.setWidth(1);
  pen2.setColor(QColor(18,232,175));
  line2->setPen(pen2);

  //Create Center Line
  QGraphicsLineItem *line3=scene->addLine(x1,y1,x2,y2);
  QPen pen3;
  pen3.setWidth(1);
  pen3.setColor(QColor(255,255,255));
  pen3.setStyle(Qt::PenStyle(Qt::DotLine));
  line3->setPen(pen3);

  //Create X-Axis Line
  line2=scene->addLine(0,m_ymax,20,m_ymax);
  pen2.setWidth(1);
  pen2.setColor(QColor(0,155,255));
  line2->setPen(pen2);

  //Create Y-Axis Line
  line3=scene->addLine(0,m_ymax,0,m_ymax-20);
  pen3.setWidth(1);
  pen3.setColor(QColor(0,155,255));
  line3->setPen(pen3);

  //Update Scene
  scene->update();

}

//Position Date Received Create Position
void DisplayWidget::OnSendPositionData(float x1, float y1, float h1)
{

  //Change scale to visuals
  x1=m_scalex*x1;
  y1=m_ymax-m_scaley*y1;

  //Remove old items
  scene->removeItem(pos);
  scene->removeItem(pos1);
  scene->removeItem(head);

  //Add Position Circles
  pos1=scene->addEllipse(x1-12.5,y1-12.5,25,25,QPen(QColor(0,0,0)),QBrush(QColor(255,0,0,50)));
  pos=scene->addEllipse(x1-5,y1-5,10,10,QPen(QColor(0,0,0)),QBrush(QColor(247,0,0)));

  //Add Heading Line
  head=scene->addLine(x1,y1,x1+cos(h1)*10,y1-sin(h1)*10);
  //Set Heading Line Color
  QPen pen2;
  pen2.setWidth(1.5);
  pen2.setColor(QColor(255,255,255));
  head->setPen(pen2);

  //Update Scene
  scene->update();
}

void DisplayWidget::OnSendMarkerData(float x1,float y1,int id)
{
  //Remove Previous Marker items
  scene->removeItem(marker1);
  scene->removeItem(marker2);
  scene->removeItem(text);

  //Scale values for rendering
  x1=m_scalex*x1;
  y1=m_ymax-y1*m_scaley;

  //Set Circle Marker Size in Pixels
  float marker1size=25;
  float marker2size=30;

  //Create Marker Circles
  marker2 =scene->addEllipse(x1-marker2size/2,y1-marker2size/2,marker2size,marker2size,QPen(QColor(0,0,0)),QBrush(QColor(255,255,255)));
  marker1 =scene->addEllipse(x1-marker1size/2,y1-marker1size/2,marker1size,marker1size,QPen(QColor(0,0,0)),QBrush(QColor(255,0,0)));


  //Convert Id to text
  char textid[]="00";
  textid[0]='0'+id/10;
  textid[1]='0'+id%10;

  //If not double digit id display -1
  if(id<0 || id>99)
  {
    textid[0]='-';
    textid[1]='1';
  }
  //Add Marker Id as Text
  text=new QGraphicsTextItem(textid);
  text->setDefaultTextColor(QColor(255,255,255));

  //Set font and size
  QFont font=QFont("Courier",-1,QFont::Bold,false);
  text->setFont(font);

  //Set Positon of text
  text->setPos(x1-12,y1-12);

  //Add text to scene
  scene->addItem(text);

  //Update Scene
  scene->update();
}

//Position Date Received Create Position
void DisplayWidget::OnSendParkingData(int id,float x,float y,int status)
{

  //Change scale to visuals
  x=m_scalex*x;
  y=m_ymax-m_scaley*y;

  //Remove old Parking
  scene->removeItem(park);

  //Parking lot size
  float szx=20;
  float szy=20;

  //Create New Rectancle
  park=new QGraphicsRectItem(x-szx/2,y-szy/2,szx,szy);

  //Set Border Color
  park->setPen(QPen(QColor(0,0,0)));

  //Update Color based on status
  if(status==0)
  {
    //Green for empty
    park->setBrush(QColor(0,255,0));
  }
  else if (status==1)
  {
    //Red for Occupied
    park->setBrush(QColor(255,0,0));
  }
  else{
    //Yellow for others
    park->setBrush(QColor(255,255,0));
  }

  //Add Item to Scene
  scene->addItem(park);

  //Update Scene
  scene->update();
}


//Position Date Received Create Position
void DisplayWidget::OnSendObstacleData(float x,float y)
{
  //Change to Visual scale
  x=m_scalex*x;
  y=m_ymax-m_scaley*y;

  //Remove Old Item
  scene->removeItem(obs);

  //Size of Obstacle
  float sz=10;
  obs=scene->addEllipse(x-sz/2,y-sz/2,sz,sz,QPen(QColor(0,0,0)),QBrush(QColor(0,255,255)));

  //Update Scene
  scene->update();
}

void DisplayWidget::setItems()
{
  //Create Empty items for dynamic deleting

  /*Position items */

  //Create Small Circle for Position
  pos=scene->addEllipse(0,0,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  //Create Big Circle for Position
  pos1=scene->addEllipse(400,400,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  //Create heading line
  head=scene->addLine(0,0,0,0);
  //Create Marker Small Cirle

  /*Marker Items */
  marker1=scene->addEllipse(0,0,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  //Create Marker Bight circle
  marker2=scene->addEllipse(0,0,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  //Create Empty text
  text=new QGraphicsTextItem("");
  scene->addItem(text);

  /*Parking Items*/
  park=new QGraphicsRectItem(0,0,0,0);
  scene->addItem(park);

  /*Obstacle Items*/
  obs=scene->addEllipse(0,0,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,255)));

}
