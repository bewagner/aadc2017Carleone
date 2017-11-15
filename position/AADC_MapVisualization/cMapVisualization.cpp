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
* $Author:: spiesra  $  $Date:: 2017-05-22 18:08:00#$ $Rev:: 63774   $
**********************************************************************/

#include "stdafx.h"
#include "cMapVisualization.h"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, cMapVisualization);

cMapVisualization::cMapVisualization(const tChar* __info) :
QObject(),
cBaseQtFilter(__info)
{
 SetPropertyBool("DEBUG::DebutOutput", tFalse); 
 SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?"); 
 SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
  SetPropertyStr ("IndoorMap File","");
  SetPropertyBool("IndoorMap File" NSSUBPROP_FILENAME, tTrue);
  SetPropertyStr ("IndoorMap File" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "OpenDrive Files (*.xodr)");
  SetPropertyStr ("IndoorMap File" NSSUBPROP_DESCRIPTION, "Here you set the OpenDrive map file");
}

cMapVisualization::~cMapVisualization()
{
}

tHandle cMapVisualization::CreateView()
{
  // create the widget
  QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
  m_pWidget = new DisplayWidget(pWidget);
if(m_filterProperties.debugOutput) { 
 LOG_INFO(cString::Format("Creating Views")); 
 }
  // make the qt connections
  connect(this, SIGNAL(SendMapScale(float,float,float,float,float)), m_pWidget, SLOT(OnMapScale(float,float,float,float,float)));
  connect(this, SIGNAL(SendMapData(float, float,float,float)), m_pWidget, SLOT(OnMapData(float, float,float,float)));
  connect(this, SIGNAL(SendPositionData(float, float,float)), m_pWidget, SLOT(OnSendPositionData(float, float ,float)));
  connect(this, SIGNAL(SendMarkerData(float,float,int)), m_pWidget, SLOT(OnSendMarkerData(float,float,int)));
  connect(this, SIGNAL(SendParkingData(int,float,float,int)), m_pWidget, SLOT(OnSendParkingData(int,float,float,int)));
  connect(this, SIGNAL(SendObstacleData(float,float)), m_pWidget, SLOT(OnSendObstacleData(float,float)));
  //Load Map
  LoadMapValue();
  //Rende map
  CreateMap();
  return (tHandle)m_pWidget;
}

tResult cMapVisualization::ReleaseView()
{
  if (m_pWidget != NULL)
  {
    delete m_pWidget;
    m_pWidget = NULL;
  }
  RETURN_NOERROR;
}

tResult cMapVisualization::Init(tInitStage eStage, __exception)
{
  RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

  if (eStage == StageFirst)
  {
if(m_filterProperties.debugOutput) { 
 LOG_INFO(cString::Format("Creating Input pins")); 
 }

    RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
    m_PosInputSet=tFalse;
    m_TrafficSignInputSet=tFalse;
    m_ParkingInputSet=tFalse;
    m_ObstacleInputSet=tFalse;
  }
  else if (eStage == StageNormal)
  {
if(m_filterProperties.debugOutput) { 
 LOG_INFO("INIT STAGE NORMAL"); 
 }

  }
  else if (eStage == StageGraphReady)
  {
if(m_filterProperties.debugOutput) { 
 LOG_INFO(cString::Format("Graph ready")); 
 }

  }
  RETURN_NOERROR;
}

tResult cMapVisualization::CreateInputPins(__exception)
{
  //get the description manager for this filter
  cObjectPtr<IMediaDescriptionManager> pDescManager;
  RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

  tChar const * strDescPos = pDescManager->GetMediaDescription("tPosition");
  RETURN_IF_POINTER_NULL(strDescPos);
  cObjectPtr<IMediaType> pTypePos = new cMediaType(0, 0, 0, "tPosition", strDescPos, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypePos->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPos));
  RETURN_IF_FAILED(m_InputPostion.Create("Position", pTypePos, static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_InputPostion));

  tChar const * strDescTrafficSign = pDescManager->GetMediaDescription("tTrafficSign");
  RETURN_IF_POINTER_NULL(strDescTrafficSign);
  cObjectPtr<IMediaType> pTypeTrafficSign = new cMediaType(0, 0, 0, "tTrafficSign", strDescTrafficSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypeTrafficSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTrafficSign));
  RETURN_IF_FAILED(m_InputTrafficSign.Create("TrafficSign", pTypeTrafficSign, static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_InputTrafficSign));

  tChar const * strDescObstacle = pDescManager->GetMediaDescription("tObstacle");
  RETURN_IF_POINTER_NULL(strDescObstacle);
  cObjectPtr<IMediaType> pTypeObstacle = new cMediaType(0, 0, 0, "tObstacle", strDescObstacle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypeObstacle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionObstacle));
  RETURN_IF_FAILED(m_InputObstacle.Create("Obstacle", pTypeObstacle, static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_InputObstacle));

  tChar const * strDescParkingSpace = pDescManager->GetMediaDescription("tParkingSpace");
  RETURN_IF_POINTER_NULL(strDescParkingSpace);
  cObjectPtr<IMediaType> pTypeParkingSpace = new cMediaType(0, 0, 0, "tParkingSpace", strDescParkingSpace, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypeParkingSpace->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionParkingSpace));
  RETURN_IF_FAILED(m_InputParkingSpace.Create("ParkingSpace", pTypeParkingSpace, static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_InputParkingSpace));

  RETURN_NOERROR;
}

tResult cMapVisualization::Start(__exception)
{
  RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));
  RETURN_NOERROR;
}

tResult cMapVisualization::Stop(__exception)
{
  RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));
  RETURN_NOERROR;
}

tResult cMapVisualization::Shutdown(tInitStage eStage, __exception)
{
  return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult cMapVisualization::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
  if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    // something was received

    if (pSource == &m_InputPostion)
    {
      tTimeStamp tsInputTime;
      tsInputTime = pMediaSample->GetTime();
      //Process Sample
      RETURN_IF_FAILED(ProcessInputPosition(pMediaSample, tsInputTime));
    }
    else if (pSource == &m_InputTrafficSign)
    {
      tTimeStamp tsInputTime;
      tsInputTime = pMediaSample->GetTime();
      //Process Sample
      RETURN_IF_FAILED(ProcessInputTrafficSign(pMediaSample, tsInputTime));
    }
    else if (pSource == &m_InputParkingSpace)
    {
      tTimeStamp tsInputTime;
      tsInputTime = pMediaSample->GetTime();
      //Process Sample
      RETURN_IF_FAILED(ProcessInputParkingSpace(pMediaSample, tsInputTime));
    }
    else if (pSource == &m_InputObstacle)
    {
      tTimeStamp tsInputTime;
      tsInputTime = pMediaSample->GetTime();
      //Process Sample
      RETURN_IF_FAILED(ProcessInputObstacle(pMediaSample, tsInputTime));
    }
  }
  RETURN_NOERROR;
}


tResult cMapVisualization::LoadMapValue()
{
  // number of points in geometry extraction
  n_points=NPOINTS;
  if(n_points<2)
  {
    n_points=2;
  }
  //Load Map File
  cFilename fileMap = GetPropertyStr("IndoorMap File");
  //create absolute path for map file
  ADTF_GET_CONFIG_FILENAME(fileMap);
  fileMap = fileMap.CreateAbsolutePath(".");

  if (fileMap.IsEmpty())
  {
    LOG_ERROR("Map not found");
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  //Number of Roads
  n_roads=0;
  if (cFileSystem::Exists(fileMap))
  {
    cDOM oDOM;
    oDOM.Load(fileMap);

    cDOMElementRefList oElems3;
    cDOMElementRefList oElems1;
    cDOMElementRefList oElems2;
    cDOMElementRefList oElems;
    cDOMElementRefList oElems4;

    //Find Roads in map
    if(IS_OK(oDOM.FindNodes("OpenDRIVE/road", oElems3)))
    {
      int rows = static_cast<int>( oElems3.size() );
      int cols = 6;
      int row=0;
      m_Road = Mat_<float>(rows,cols);
      int r=0,z=0,p=0,th=0;
      for (cDOMElementRefList::iterator itElem = oElems3.begin(); itElem != oElems3.end(); ++itElem)
      {
        m_Road(r,4) = float((*itElem)->GetAttribute("id","0").AsFloat64());
        m_Road(r,5) = float((*itElem)->GetAttribute("junction","0").AsFloat64());

        char a[10];
        int val=m_Road(r,4);
        sprintf(a,"%d",val);
        char Q[100];
        strcpy(Q,"OpenDRIVE/road");
        strcat(Q,"[@id=\"");
        strcat(Q,a);
        strcat(Q,"\"]");
        
        char Q1[200];
        strcpy(Q1,Q);
        strcat(Q1,"/link/predecessor");
        //For Each road find predecessor
        if(IS_OK(oDOM.FindNodes(Q1,oElems1)))
        {
          for (cDOMElementRefList::iterator itElem = oElems1.begin(); itElem != oElems1.end(); ++itElem)
          {
            m_Road(r,0) = float((*itElem)->GetAttribute("elementId","0").AsFloat64());
            if((*itElem)->GetAttribute("contactPoint","0")=="start")
            {
              m_Road(r,1) = 0.0;
            }
            else
            {
              m_Road(r,1) = 1.0;
            }
          }
        }
        else
        {
          m_Road(r,0) = -1.0;
          m_Road(r,1) = -1.0;
        }

        char Q2[200];
        strcpy(Q2,Q);
        strcat(Q2,"/link/successor");
        //For Each Road Find successor
        if(IS_OK(oDOM.FindNodes(Q2,oElems2)))
        {
          for (cDOMElementRefList::iterator itElem = oElems2.begin(); itElem != oElems2.end(); ++itElem)
          {
            m_Road(r,2) = float((*itElem)->GetAttribute("elementId","0").AsFloat64());
            if((*itElem)->GetAttribute("contactPoint","0")=="start")
            {
              m_Road(r,3) = 0.0;
            }
            else
            {
              m_Road(r,3) = 1.0;
            }
          }
        }
        else
        {
          m_Road(r,2) = -1.0;
          m_Road(r,3) = -1.0;
        }

        char Q3[200];
        strcpy(Q3,Q);
        strcat(Q3,"/planView/geometry");
        //Create Geometry matrix
        if(z==0)
        {
          if(IS_OK(oDOM.FindNodes("OpenDRIVE/road/planView/geometry", oElems)))
          {

            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
              row++;

            }
          }
          m_Mapv = Mat_<float>(row,9);
        }


        if(IS_OK(oDOM.FindNodes(Q3, oElems)))
        {

          //For each Geometry in road find attributes

          for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
          {
            m_Mapv(z,0) = float((*itElem)->GetAttribute("s","0").AsFloat64());
            m_Mapv(z,1) = float((*itElem)->GetAttribute("x","0").AsFloat64());
            m_Mapv(z,2) = float((*itElem)->GetAttribute("y","0").AsFloat64());
            m_Mapv(z,3) = float((*itElem)->GetAttribute("hdg","0").AsFloat64());

            m_Mapv(z,4) = float((*itElem)->GetAttribute("length","0").AsFloat64());
            m_Mapv(z,5) = m_Road(r,4);
            m_Mapv(z,6) = m_Road(r,0);
            m_Mapv(z,7) = m_Road(r,2);
            m_Mapv(z,8) = m_Road(r,5);

            if(m_Road(r,5)== -1)
            {
              n_roads++;
            }
            z++;
          }
        }


        char Q4[200];
        strcpy(Q4,Q);
        strcat(Q4,"/planView/geometry/paramPoly3");
        //Find number of points to be created
        if(p==0)
        {
          m_Points = Mat_<float>(row*(n_points),9);
          m_Points=0;
        }

        if(IS_OK(oDOM.FindNodes(Q4, oElems4)))
        {

          float au=0,av=0,bu=0,bv=0,cu=0,cv=0,du=0,dv=0;
          //Find Attributes of each cubic polynomial in geometry
          for (cDOMElementRefList::iterator itElem = oElems4.begin(); itElem != oElems4.end(); ++itElem)
          {

            au=float((*itElem)->GetAttribute("aU","0").AsFloat64());
            bu=float((*itElem)->GetAttribute("bU","0").AsFloat64());
            cu=float((*itElem)->GetAttribute("cU","0").AsFloat64());
            du=float((*itElem)->GetAttribute("dU","0").AsFloat64());

            av=float((*itElem)->GetAttribute("aV","0").AsFloat64());
            bv=float((*itElem)->GetAttribute("bV","0").AsFloat64());
            cv=float((*itElem)->GetAttribute("cV","0").AsFloat64());
            dv=float((*itElem)->GetAttribute("dV","0").AsFloat64());
            float u=0,v=0,u1=0,v1=0;

            for(int pnt=0;pnt<n_points;pnt++)
            {
              float ds=float(pnt)/(n_points-1);
              u=CubicPoly(au,bu,cu,du,ds);
              v=CubicPoly(av,bv,cv,dv,ds);
              u1=RotateCWX(u,v,m_Mapv(th,3));
              v1=RotateCWY(u,v,m_Mapv(th,3));
              m_Points(p,0)=float(pnt);//Road point number
              m_Points(p,1)=u1+m_Mapv(th,1);//Map x after translation and rotation
              m_Points(p,2)=v1+m_Mapv(th,2);//Map y after translation and rotation
              m_Points(p,3)=m_Road(r,4); //Road Id
              m_Points(p,4)=m_Road(r,5); //Junction Id
              m_Points(p,5)=m_Road(r,0); //Predecessor
              m_Points(p,6)=m_Road(r,2); //Successor
              m_Points(p,7)=0; //Heading
              m_Points(p,8)=0; //Length
              p++;
            }
            th++;
          }
        }
        else
        {
          z++;
        }

        r++;
      }

    }

    for (int i = 0; i < m_Points.rows; i++) {
      if(n_points-1==m_Points(i,0))
      {//if last point of path, use previous values
        m_Points(i,7)=m_Points(i-1,7);
        m_Points(i,8)=m_Points(i-1,8);
      }
      else{//Calculate heading and length
        float tx1=m_Points(i,1);
        float ty1=m_Points(i,2);
        float tx2=m_Points(i+1,1);
        float ty2=m_Points(i+1,2);
        m_Points(i,7)=atan2(ty2-ty1,tx2-tx1);
        m_Points(i,8)=sqrt((ty2-ty1)*(ty2-ty1)+(tx2-tx1)*(tx2-tx1));
      }
    }

    int t_p=(m_Road.rows-n_roads)*n_points/2;
    //Find number of junction roads
    int n_jun=m_Road.rows-n_roads;

    //Create road points with 1 junction road
    m_rPoints=Mat_<float>(m_Points.rows-t_p,9);
    m_rPoints=0;
    //Store junction ids
    m_Junc=Mat_<float>(n_jun,4);
    m_Junc=0;

    //Center road point counter
    int tjun=0;
    //Store Center roads
    int t_j=0;
    for (int jun=0;jun<m_Points.rows;jun++)
    {
      //If the road is not a junction store it in new matrix
      if(m_Points(jun,4)==-1)
      {
        //Copy all data of points
        m_rPoints(tjun,0)=m_Points(jun,0);
        m_rPoints(tjun,1)=m_Points(jun,1);
        m_rPoints(tjun,2)=m_Points(jun,2);
        m_rPoints(tjun,3)=m_Points(jun,3);
        m_rPoints(tjun,4)=m_Points(jun,4);
        m_rPoints(tjun,5)=m_Points(jun,5);
        m_rPoints(tjun,6)=m_Points(jun,6);
        m_rPoints(tjun,7)=m_Points(jun,7);
        m_rPoints(tjun,8)=m_Points(jun,8);
        //Separate counter for road points
        tjun++;
      }
      else
      {
        //If its a junction store the position, predecessor and successor roads
        if(m_Points(jun,0)==0)
        {
          m_Junc(t_j,0)=m_Points(jun,5);
          m_Junc(t_j,1)=m_Points(jun,6);
          m_Junc(t_j,2)=jun;
          //Counter for number of junctions
          t_j++;
        }
      }
    }
    //Find the junction roads with same predecessor and successor, store it in the 4th column
    for(int j=0;j<m_Junc.rows;j++)
    {
      for(int ij=0;ij<m_Junc.rows;ij++)
      {
        //if predecessor and successor are same
        if(m_Junc(j,0)==m_Junc(ij,0) && m_Junc(j,1)==m_Junc(ij,1))
        {
          //Store location
          m_Junc(j,3)=m_Junc(ij,2);
        }
        else if(m_Junc(j,0)==m_Junc(ij,1) && m_Junc(j,1)==m_Junc(ij,0))
        {
          //Store location
          m_Junc(j,3)=m_Junc(ij,2);
        }
      }
    }
    t_j=0;
    //Fill the junction points as an average of the two roads
    for (int jun=0;jun<m_Junc.rows;jun++)
    {
      if( m_Junc(jun,2) != m_Junc(jun,3) )
      {
        //Take the matched position from junction matrix
        int pos1=m_Junc(jun,2);
        int pos2=m_Junc(jun,3);
        for(int i=0;i<n_points;i++)
        {
          //Take average of x and y position between 2 junction roads
          m_rPoints(tjun,0)=i;
          //m_Points(pos1+i,0)
          m_rPoints(tjun,1)=(m_Points(pos1+i,1)+m_Points(pos2+i,1))/2;
          m_rPoints(tjun,2)=(m_Points(pos1+i,2)+m_Points(pos2+i,2))/2;
          //Copy First points data
          m_rPoints(tjun,3)=m_Points(pos2+i,3);
          m_rPoints(tjun,4)=m_Points(pos2+i,4);
          m_rPoints(tjun,5)=m_Points(pos2+i,5);
          m_rPoints(tjun,6)=m_Points(pos2+i,6);
          m_rPoints(tjun,7)=m_Points(pos2+i,7);
          m_rPoints(tjun,8)=m_Points(pos2+i,8);
          tjun++;
        }
      }
    }
  }
  else
  {
    LOG_ERROR("Map file does not exist");
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  RETURN_NOERROR;
}

tResult cMapVisualization::CreateMap()
{
  //Find maximum and minimum coordinates of x and y
  float xmax=-999999999.0f;
  float ymax=-999999999.0f;
  float xmin=9999999999.9f;
  float ymin=999999999.9f;
  //Print points for Debug
  for (int i = 0; i < m_rPoints.rows; i++) {
    if(m_rPoints(i,1)>xmax)
    {
      xmax=m_rPoints(i,1);
    }
    if(m_rPoints(i,1)<xmin)
    {
      xmin=m_rPoints(i,1);
    }
    if(m_rPoints(i,2)>ymax)
    {
      ymax=m_rPoints(i,2);
    }
    if(m_rPoints(i,2)<ymin)
    {
      ymin=m_rPoints(i,2);
    }
  }
  //Scale to map size
  float scx=SCALEX;
  float scy=SCALEY;
  //Shift center point to Anchor zero
  for(int var=0;var<m_rPoints.rows;var++)
  {
    m_rPoints(var,1)=(m_rPoints(var,1)-xmin)*scx+LANEWIDTH;
    m_rPoints(var,2)=(m_rPoints(var,2)-ymin)*scy+LANEWIDTH;
  }
  x_min=0;
  y_min=0;
  x_max=(xmax-xmin)*scx+LANEWIDTH;
  y_max=(ymax-ymin)*scy+LANEWIDTH;
  float aspRatio=(y_max-y_min)/(x_max-x_min);
  m_scalex=400.0/(x_max-x_min);//Find scaling for rendering
  m_scaley=400.0*aspRatio/(y_max-y_min);//Find scaling for rendering
if(m_filterProperties.debugOutput) { 
 LOG_INFO(cString::Format("Minmax: mnx %.3f mny %.3f mxx %.3f mxy %.3f", x_min,y_min,x_max,y_max)); 
 }

if(m_filterProperties.debugOutput) { 
 LOG_INFO(cString::Format("Scales: scx %.3f scy %.3f sx %.3f sy %.3f", m_scalex, m_scaley, m_shiftx, m_shifty)); 
 }
  emit SendMapScale(static_cast<float>(x_max*m_scalex), static_cast<float>(y_max*m_scaley),static_cast<float>(m_scalex), static_cast<float>(m_scaley),static_cast<float>(LANEWIDTH));

  for (int i = 0; i < m_rPoints.rows-1; i++) {
    float tx1=m_rPoints(i,1)*m_scalex;
    float ty1=m_rPoints(i,2);
    float tx2=m_rPoints(i+1,1)*m_scalex;
    float ty2=m_rPoints(i+1,2);
    if(n_points-1==m_rPoints(i,0))
    {
      tx2=m_rPoints(i-1,1)*m_scalex;
      ty2=m_rPoints(i-1,2);
    }
    ty1=y_max-ty1;//Invert y axis
    ty2=y_max-ty2;//Invert y axis
    ty1=ty1*m_scaley;//Scale to Visual axis
    ty2=ty2*m_scaley;//Scale to Visual axis
    emit SendMapData(static_cast<float>(tx1), static_cast<float>(ty1),static_cast<float>(tx2), static_cast<float>(ty2));
  }
  RETURN_NOERROR;
}

tResult cMapVisualization::ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{

  tFloat32 f32x = 0;
  tFloat32 f32y = 0;
  tFloat32 f32radius = 0;
  tFloat32 f32speed = 0;
  tFloat32 f32heading = 0;

  {   __adtf_sample_read_lock_mediadescription(m_pDescriptionPos,pMediaSampleIn,pCoderInput);
    // get IDs
    if (!m_PosInputSet)
    {
      pCoderInput->GetID("f32x", m_szF32X);
      pCoderInput->GetID("f32y", m_szF32Y);
      pCoderInput->GetID("f32radius", m_szF32Radius);
      pCoderInput->GetID("f32speed", m_szF32Speed);
      pCoderInput->GetID("f32heading", m_szF32Heading);
      m_PosInputSet=tTrue;
    }

    pCoderInput->Get(m_szF32X, (tVoid*)&f32x);
    pCoderInput->Get(m_szF32Y, (tVoid*)&f32y);
    pCoderInput->Get(m_szF32Radius, (tVoid*)&f32radius);
    pCoderInput->Get(m_szF32Speed, (tVoid*)&f32speed);
    pCoderInput->Get(m_szF32Heading, (tVoid*)&f32heading);

  }
  emit SendPositionData(static_cast<float>(f32x), static_cast<float>(f32y),static_cast<float>(f32heading));
  RETURN_NOERROR;
}

tResult cMapVisualization::ProcessInputTrafficSign(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{
  tInt16 i16Id=0;
  tFloat32 f32x = 0;
  tFloat32 f32y = 0;
  tFloat32 f32angle = 0;

  {   __adtf_sample_read_lock_mediadescription(m_pDescriptionTrafficSign,pMediaSampleIn,pCoderInput);
    // get IDs
    if (!m_TrafficSignInputSet)
    {
      pCoderInput->GetID("i16Identifier", m_tsI16id);
      pCoderInput->GetID("f32x", m_tsF32X);
      pCoderInput->GetID("f32y", m_tsF32Y);
      pCoderInput->GetID("f32angle", m_tsF32Angle);
      m_TrafficSignInputSet=tTrue;
    }
    pCoderInput->Get(m_tsI16id, (tVoid*)&i16Id);
    pCoderInput->Get(m_tsF32X, (tVoid*)&f32x);
    pCoderInput->Get(m_tsF32Y, (tVoid*)&f32y);
    pCoderInput->Get(m_tsF32Angle, (tVoid*)&f32angle);
  }
  emit SendMarkerData(static_cast<float>(f32x), static_cast<float>(f32y),static_cast<int>(i16Id));
  RETURN_NOERROR;
}

tResult cMapVisualization::ProcessInputParkingSpace(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{
  tInt16 i16Id=0;
  tFloat32 f32x = 0;
  tFloat32 f32y = 0;
  tUInt16 ui16Status = 0;

  {   __adtf_sample_read_lock_mediadescription(m_pDescriptionParkingSpace,pMediaSampleIn,pCoderInput);
    // get IDs
    if (!m_ParkingInputSet)
    {
      pCoderInput->GetID("i16Identifier", m_parkingI16Id);
      pCoderInput->GetID("f32x", m_parkingF32X);
      pCoderInput->GetID("f32y", m_parkingF32Y);
      pCoderInput->GetID("ui16Status", m_parkingUI16Status);
      m_ParkingInputSet=tTrue;
    }
    pCoderInput->Get(m_parkingI16Id, (tVoid*)&i16Id);
    pCoderInput->Get(m_parkingF32X, (tVoid*)&f32x);
    pCoderInput->Get(m_parkingF32Y, (tVoid*)&f32y);
    pCoderInput->Get(m_parkingUI16Status, (tVoid*)&ui16Status);
  }
  emit SendParkingData(static_cast<int>(i16Id),static_cast<float>(f32x), static_cast<float>(f32y),static_cast<int>(ui16Status));
  RETURN_NOERROR;
}

tResult cMapVisualization::ProcessInputObstacle(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{
  tFloat32 f32x = 0;
  tFloat32 f32y = 0;
  {
    __adtf_sample_read_lock_mediadescription(m_pDescriptionObstacle,pMediaSampleIn,pCoderInput);
    // get IDs
    if (!m_ObstacleInputSet)
    {
      pCoderInput->GetID("f32x", m_obstacleF32X);
      pCoderInput->GetID("f32y", m_obstacleF32Y);
      m_ObstacleInputSet=tTrue;
    }
    pCoderInput->Get(m_obstacleF32X, (tVoid*)&f32x);
    pCoderInput->Get(m_obstacleF32Y, (tVoid*)&f32y);
  }
  emit SendObstacleData(static_cast<float>(f32x), static_cast<float>(f32y) );
  RETURN_NOERROR;
}

float cMapVisualization::CubicPoly(float a1,float b1,float c1, float d1, float ds)
{

  return (a1+b1*ds+c1*pow(ds,2.0f)+d1*pow(ds,3.0f));
}

float cMapVisualization::RotateCWX(float u2,float v2, float hdg2)
{
  return (u2*cos(hdg2)-v2*sin(hdg2));
}


float cMapVisualization::RotateCWY(float u1,float v1, float hdg1)
{
  return (u1*sin(hdg1)+v1*cos(hdg1));
}
