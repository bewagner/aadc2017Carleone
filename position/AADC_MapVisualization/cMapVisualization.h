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
* $Author:: kuckal   $  $Date:: 2017-05-22 09:58:28#$ $Rev:: 63664   $
**********************************************************************/


#ifndef _AADC_MAP_VISUALIZATION
#define _AADC_MAP_VISUALIZATION

#include "stdafx.h"
#include "displaywidget.h"

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180
#define FACTORRADTODEG 180.f/M_PI

#define OID_ADTF_FILTER_DEF                "adtf.aadc.MapVisualization"
#define ADTF_FILTER_DESC                   "AADC Map Visualization"
#define ADTF_FILTER_VERSION_SUB_NAME       "Map Visualization"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL   "AADC MapVisualization"
#define ADTF_FILTER_VERSION_STRING         "1.0.0"
#define ADTF_FILTER_VERSION_Major          1
#define ADTF_FILTER_VERSION_Minor          0
#define ADTF_FILTER_VERSION_Build          0
#define ADTF_FILTER_VERSION_LABEL          "Map Visualization for ADTF."
#define ADTF_CATEGORY OBJCAT_Application

//Map features
#define SCALEX 0.125
#define SCALEY 0.125
#define LANEWIDTH 0.5
#define NPOINTS 20

/*! @defgroup MapVisualization Map Visualization
*  @{
*
* This filter reads and renders an OpenDrive map, and it also displays the input position when provided.
* The filter will also provide means for checking positioning of the traffic sign, obstacle and parking space measurements.
*
*  \image html MapVisualization.PNG "Plugin Map Visualization"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li OpenCV v.3.2.0
* \li Qt v.4.7.1
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Map File<td>Provides the location of Open Drive map file. ".xodr" extension<td>.
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>Position<td>Current map position<td>tPosition
* <tr><td>TrafficSign<td>Located Traffic signs<td>tTrafficSign
* <tr><td>Obstacle<td>Located Obstacles<td>tObstacle
* <tr><td>ParkingSpace<td>Located Parking Spaces<td>tParkingSpace
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/position/AADC_MapVisualization
* <tr><td>Filename<td>aadc_mapVisualization.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*!
* This is the main class of the Map Visualization Filter
*/
class cMapVisualization : public QObject, public cBaseQtFilter
{
	/*! This macro does all the plugin setup stuff
	* Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
	*/
	ADTF_FILTER_VERSION(
		OID_ADTF_FILTER_DEF,
		ADTF_FILTER_DESC,
		ADTF_CATEGORY,
		ADTF_FILTER_VERSION_SUB_NAME,
		ADTF_FILTER_VERSION_Major,
		ADTF_FILTER_VERSION_Minor,
		ADTF_FILTER_VERSION_Build,
		ADTF_FILTER_VERSION_LABEL);

    Q_OBJECT

public: // construction
    cMapVisualization(const tChar *);
    virtual ~cMapVisualization();

    // overrides cFilter
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    virtual tResult Start(__exception = NULL);
    virtual tResult Stop(__exception = NULL);
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

    /*!Handles all the input from the arduino*/
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    //Map maximum and minimum values
    float x_min,y_min,x_max,y_max;

    //Scaling for visuals
    float m_scalex,m_scaley,m_shiftx,m_shifty;

protected: // Implement cBaseQtFilter

    /*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

private:
 struct filterProperties 
 { 
 tBool debugOutput;
 }; 
 filterProperties m_filterProperties;

    //Scaling for visuals
    cInputPin m_InputPostion;
    cInputPin m_InputTrafficSign;
    cInputPin m_InputObstacle;
    cInputPin m_InputParkingSpace;

    //Position input
    tBool m_PosInputSet;
    tBufferID m_szF32X,m_szF32Y,m_szF32Radius,m_szF32Speed,m_szF32Heading;

    //Traffic Sign input
    tBool m_TrafficSignInputSet;
    tBufferID m_tsI16id,m_tsF32X,m_tsF32Y,m_tsF32Angle;

    //Parking Input
    tBool m_ParkingInputSet;
    tBufferID m_parkingI16Id,m_parkingF32X,m_parkingF32Y,m_parkingUI16Status;

    //Obstacle Input
    tBool m_ObstacleInputSet;
    tBufferID m_obstacleF32X,m_obstacleF32Y;

    //Media Description
    cObjectPtr<IMediaTypeDescription> m_pDescriptionPos;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionTrafficSign;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionObstacle;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionParkingSpace;

    tResult CreateInputPins(__exception = NULL);

    //Matrices for Storing map details
    Mat_<float> m_Mapv;//Road Geometry Details
    Mat_<float> m_Points,m_rPoints,m_Junc;//Map Points without left and right lane
    Mat_<float> m_Road;//Road id, Predecessor and successor
    int n_points;
    int n_roads;
    //Loading map from open drive file
    tResult LoadMapValue();
    //Render map
    tResult CreateMap();

    tResult ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);
    tResult ProcessInputTrafficSign(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);
    tResult ProcessInputParkingSpace(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);
    tResult ProcessInputObstacle(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);

    float CubicPoly(float a1,float b1,float c1, float d1, float ds);
    float RotateCWX(float u2,float v2, float hdg2);
    float RotateCWY(float u1,float v1, float hdg1);
signals:
    void SendMapData(float x1,float y1,float x2,float y2);
    void SendMapScale(float x1, float y1,float x2,float y2,float laneWidth);
    void SendPositionData(float x, float y,float h);
    void SendMarkerData(float x,float y,int id);
    void SendParkingData(int id,float x,float y,int status);
    void SendObstacleData(float x,float y);
};

#endif
