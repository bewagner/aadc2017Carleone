/**
   Copyright (c)
   Audi Autonomous Driving Cup. All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
   4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   **********************************************************************
   * $Author:: spiesra $  $Date:: 2017-05-09 08:15:08#$ $Rev:: 62948   $
   **********************************************************************/
#ifndef _CLASSIFICATIONPOSTPROCESSING_FILTER_HEADER_
#define _CLASSIFICATIONPOSTPROCESSING_FILTER_HEADER_

#include "functions.h"
#include "stdafx.h"

#include "user_classification_structs.h"
#include <iostream>
#include "ADTF_OpenCV_helper.h"
#include <fstream>
#include <algorithm>    // std::sort
#include <vector> 
#include <deque>


#define OID_ADTF_CLASSIFICATIONPOSTPROCESSING  "adtf.aadc.user_classification_Postprocessing"

#define OID_ADTF_BFFT_FILTER_DEF "bfft.user_classification_Postprocessing" //unique for a filter
#define ADTF_BFFT_FILTER_DESC "AADC User Classification Postprocessing"  //this appears in the Component Tree in ADTF
#define ADTF_BFFT_FILTER_VERSION_SUB_NAME "ClassificationPostprocessingFilter"//must match with accepted_version_...
#define ADTF_BFFT_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_BFFT_FILTER_VERSION "1.0.0"//version string
#define ADTF_BFFT_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_BFFT_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_BFFT_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_BFFT_FILTER_VERSION_LABEL "A small OpenCV Template Filter from BFFT.\n$Rev:: 62948   $\nCopyright (c) 2016 BFFT Gesellschaft fuer Fahrzeugtechnik mbH. All rights reserved."

/*! @defgroup OpenCVTemplateFilter
 *  @{
 *
 *  \image html User_OpenCVTemplate.PNG "Plugin OpenCV Template Filter"
 * This is a small OpenCV template which can be used by the AADC teams for their own filter implementations for image processing.
 *
 * This plugin needs the following libraries:
 * \li OpenCV  v.3.2.0
 *
 * <b> Filter Properties</b>
 * <table>
 * <tr><th>Property<th>Description<th>Default
 * </table>
 *
 * <b>Output Pins</b>
 * <table>
 * <tr><th>Pin<th>Description<th>MajorType<th>SubType
 * <tr><td>Video_Output<td>Video Pin for data from camera<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
 *</table>
 *
 * <b>Input Pins</b>
 * <table>
 * <tr><th>Pin<th>Description<th>MajorType<th>SubType
 * <tr><td>Video_Input<td>Video Pin for data from camera<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
 * </table>
 *
 * <b>Plugin Details</b>
 * <table>
 * <tr><td>Path<td>src/aadcUser/AADC_ClassificationPostprocessing
 * <tr><td>Filename<td>user_ClassificationPostprocessing.plb
 * <tr><td>Version<td>1.0.0
 * </table>
 *
 */

//!  Template filter for OpenCV Image Processing
/*!
 * This class is the main class of the OpenCV Template Filter and can be used as template for user specific image processing filters
 */
class cClassificationPostprocessing : public adtf::cFilter
{

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_BFFT_FILTER_DEF,
						ADTF_BFFT_FILTER_DESC,
						adtf::OBJCAT_Auxiliary,
						ADTF_BFFT_FILTER_VERSION_SUB_NAME,
						ADTF_BFFT_FILTER_VERSION_Major,
						ADTF_BFFT_FILTER_VERSION_Minor,
						ADTF_BFFT_FILTER_VERSION_Build,
						ADTF_BFFT_FILTER_VERSION_LABEL
		);

protected:

	cCriticalSection CritCrossingOut;
    cCriticalSection CritServerRunningOut;
    cCriticalSection CritBrakeLightOut;
    cCriticalSection CritZebraOut;
    cCriticalSection CritKidOut;
    cCriticalSection CritPersonOnStreetOut;


	/* cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue1; */
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolean1;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolean2;

	/*  */
	cObjectPtr<IMediaTypeDescription> m_pDescriptionInt;

	// INPUT PINS

	/*! input for classification*/
	cInputPin        m_oClassificationPin;


	// OUTPUT PINS
	
	/*! Output pin für zebra Streifen */ 
	cOutputPin	m_oOutputZebraStreifen; //tSignalValue

	tBufferID	m_szIDZebraStreifen;
	tBufferID	m_szIDZebraStreifenTimeStamp;

	tBool		m_szIDZebraStreifenSet;
	
	/*! Output pin for the crossing output */ 
	cOutputPin	m_oOutputCrossing; //tSignalValue

	tBufferID	m_szIDCrossing;
	tBufferID	m_szIDCrossingTimeStamp;

	tBool		m_szIDCrossingSet;

	/*! Output pin when seeing a kid */ 
	cOutputPin	m_oOutputKidSeen; //tSignalValue
	tBufferID	m_szIdKidSeen;

	tBufferID	m_szIDKidSeenTimeStamp;
	tBool		m_szIDKidSeenSet;

	/*! Output pin when person seen on street */ 
	cOutputPin	m_oOutputPersonOnStreetSeen; //tSignalValue
	tBufferID	m_szIdPersonOnStreetSeen;

	tBufferID	m_szIDPersonOnStreetSeenTimeStamp;
	tBool		m_szIDPersonOnStreetSeenSet;

	/*! Output pin telling us that classifications are arriving. This means, we can start driving. */ 
	cOutputPin	m_oOutputServerIsRunning; //tSignalValue
	tBufferID	m_szIdServerIsRunning;
	tBufferID	m_szIDServerIsRunningTimeStamp;
	tBool		m_szIDServerIsRunningSet;

	/*! Output pin when Brakelight shall be used */ 
	cOutputPin	m_oOutputBrakeLight; //tSignalValue
	tBufferID	m_szIdBrakeLight;
	tBufferID	m_szIDBrakeLightTimeStamp;
	tBool		m_szIDBrakeLightSet;





public:
	/*! default constructor for template class
	  \param __info   [in] This is the name of the filter instance.
	*/
	cClassificationPostprocessing(const tChar* __info);


	/*! default destructor */
	virtual ~cClassificationPostprocessing();


	//tResult ProcessInputDepth(IMediaSample* pSample, std::vector<cnnClassificationResult>::iterator);
	tBool IsAdult(std::vector<cnnClassificationResult>::iterator);
	/*

	 *	    called automatically by changing the filters state and needs
	 *	    to be overwritten by the special filter.
	 *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
	 *
	 *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
	 *        If not using the cException smart pointer, the interface has to
	 *        be released by calling Unref().
	 *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
	 *    \return Standard Result Code.
	 */
	tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

	/*!
	 *   Implements the default cFilter state machine call. It will be
	 *   called automatically by changing the filters state and needs
	 *   to be overwritten by the special filter.
	 *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
	 *
	 *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
	 *                                   If not using the cException smart pointer, the interface has to
	 *                                   be released by calling Unref().
	 *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
	 *   \result Returns a standard result code.
	 *
	 */
	
	tResult TransmitCrossing(const tInt32 cars, tTimeStamp timestamp);
	

	tResult TransmitKid(const tBool kid, tTimeStamp timestamp);
	
	tResult TransmitServerRunning(const tBool running, tTimeStamp timestamp);

	tUInt32 GetTime();

	vector<vector<cnnClassificationResult> > TrafficLocation(std::vector<cnnClassificationResult> resultVector, tFloat32 left_percent, tFloat right_percent);

	std::vector<cnnClassificationResult> classificationResults;


	tBool Search4Kids(vector<cnnClassificationResult> classifications);

	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

	tBool PersonTowardsStreet(vector<vector<cnnClassificationResult> >);
	
	tBool FindPersonOnStreet(vector<vector<cnnClassificationResult> >);

	tResult TransmitZebra(const tBool zebra, tTimeStamp timestamp);
    
    tFloat32 DistanceToObject(tFloat32 ymax);
    
    tResult TransmitPersonOnStreet(const tBool personOnStreet, tTimeStamp timestamp);

	tInt32 AnalyseCrossing(vector<vector<cnnClassificationResult> > CarsAndPersons);

	vector<vector<cnnClassificationResult> > CarsNPersonsLocationVector;

	vector<vector<cnnClassificationResult> > PersonOnStreetVector;

    unsigned short Distance2Object(cv::Mat& m);

	tResult PersonOrientation(vector<vector<cnnClassificationResult> > results);

	tResult TransmitBrakeLight(const tBool brake, tTimeStamp timestamp);
	tResult AverageDepthInfoOverTime(std::vector<cv::Mat> depthImages, tFloat32 yMin, tFloat32 xMin, tFloat32 yMax, tFloat32 xMax);

	/*! This Function will be called by all pins the filter is registered to.
	 *   \param [in] pSource Pointer to the sending pin's IPin interface.
	 *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
	 *   \param [in] nParam1 Optional integer parameter.
	 *   \param [in] nParam2 Optional integer parameter.
	 *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
	 *   \return   Returns a standard result code.
	 *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
	 *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
	 */
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	
	tResult PropertyChanged(const tChar* strName);
	/*! Implements the default cFilter state machine calls. It will be
	 *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
	 *    and can be overwritten by the special filter.
	 *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
	 *        If not using the cException smart pointer, the interface has to
	 *        be released by calling Unref().
	 *    \return Standard Result Code.
	 *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
	 *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
	 *
	 */
	tResult Start(ucom::IException** __exception_ptr = NULL);

	/*!  Implements the default cFilter state machine calls. It will be
	 *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
	 *   and can be overwritten by the special filter.
	 *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
	 *   If not using the cException smart pointer, the interface has to
	 *   be released by calling Unref().
	 *   \return Standard Result Code.
	 *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
	 *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
	 */
	tResult Stop(ucom::IException** __exception_ptr = NULL);


private: // Private Methods

	/*! function to set the m_sProcessFormat and the  m_sInputFormat variables
	 *   \param pFormat the new format for the input pin
	 *   \return Standard Result Code.
	 */
	tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    


	/*! function to process the mediasample
	 *   \param pSample the new media sample
	 *   \return Standard Result Code.
	 */
	tResult ProcessVideo(IMediaSample* pSample);

	/*! bitmap format of input pin */
	tBitmapFormat m_sInputFormat;


	struct filterProperties
	{
		tFloat64 minProbability;
		tBool kidClassification;
		tInt CrossingClassificationQueueSize;		
		tInt personOrientationThreshold;
		tFloat32	LeftSidePercentage;
		tFloat32	RightSidePercentage;
		tFloat32	PersonOnStreetLeftSidePercentage;
		tFloat32	PersonOnStreetRightSidePercentage;
        tFloat32 PersonOnStreetEmergencyStopDistance;
        tBool debugOutput;
    };
	filterProperties m_filterProperties;


	tBitmapFormat m_sInputFormatDepthImage;
   
	tBool        m_bFirstFrameDepthimage;
	cv::Mat		m_matImageDepth;

	tInt		m_nImagecutWidthLeft;
	tInt		m_nImagecutWidthRight;
	tInt		m_nImagecutHeightUp;
	tInt		m_nImagecutHeightDown;
	float MedianMat(cv::Mat Input, int nVals);
	cv::Mat m_CurrentdepthImage;
	std::vector<cv::Mat> DepthImageVector;	
	const char* picture;
	std::vector<cnnClassificationResult>::iterator m_PersonTowardsStreet;
	vector<tFloat32> min_max_mean;
	std::vector<tFloat32> ortho;

	vector<tFloat32> kid_val;
	
	vector<tFloat32> lower_bound;

	//tInt16 person_side_orientation_array[3][3];
	tBool changed;


	std::deque<tInt32> left;

	std::deque<tInt32> right;

	std::deque<tInt32> center;
	
	tInt32 left_sum;	
	tInt32 right_sum;	
	tInt32 center_sum;	

};

/** @} */ // end of group

#endif  //_CLASSIFICATIONPOSTPROCESSING_FILTER_HEADER_
