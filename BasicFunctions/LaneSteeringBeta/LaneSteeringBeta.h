/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-22 18:08:00#$ $Rev:: 63774   $
**********************************************************************/
#ifndef _LANESTEERINGBETA_FILTER_HEADER_
#define _LANESTEERINGBETA_FILTER_HEADER_

#include "ADTF_OpenCV_helper.h"
#include <fstream>


#define OID_ADTF_FILTER_DEF "adtf.LaneSteeringBeta" //unique for a filter
#define ADTF_FILTER_DESC "LaneSteeringBeta"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "OpenCVLaneSteeringBeta"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small Lane DetectionFilter"

/*! @defgroup LaneDetection Lane Detection
*  @{
*
*  \image html LaneDetction.PNG "Plugin Lane Detection Filter"
*         
* This filter does a very simple detection of lanes in the input image and can be used by the teams as a startup for their own implementation.
* It does the following subsequent steps:
* \li binarize input image with threshold set in the properties "Threshold for image binarization"
* \li calculate the horizontal lines where to search for the Lanes. This is defined by the ROI and the number "Algorithm::Detection Lines" set in the filter properties
* \li iterate through the detection lines and find transitions with high contrasts. For each detection line we iterate from left to right and search for lines within the maximum and minimum width defined in "Algorithm::Minimum Line Width" and "Algorithm::Maximum Line Width"
* \li# all the found linepoints are added to one vector
* # suggested: do a classification which point is left line, middle line, right line
* \li# suggested: calculate a clothoide for each line
* \li# suggested: calculate a clothoide for the car
* 
* This plugin needs the following libraries:
* \li OpenCV  v.3.2.0
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>ROI::XOffset <td>X Offset for Region of Interest Rectangular<td>0
* <tr><td>ROI::YOffset <td>Y Offset for Region of Interest Rectangular<td>500
* <tr><td>ROI::Width <td>Width of the Region of Interest Rectangular<td>1280
* <tr><td>ROI::Height <td>Height of the Region of Interest Rectangular<td>200
* <tr><td>Algorithm::Detection Lines<td>number of detection lines searched in ROI<td>10
* <tr><td>Algorithm::Minimum Line Width<td>Minimum Line Width in Pixel<td>10
* <tr><td>Algorithm::Maximum Line Width<td>Maximum Line Width in Pixel<td>30
* <tr><td>Algorithm::Minimum Line Contrast<td>Mimimum line contrast in gray Values<td>50
* <tr><td>Algorithm::Image Binarization Threshold<td>Threshold for image binarization<td>180
* </table>
*
* <b>Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_Output_Debug<td>Video Output for debugging<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>GCL<td>GCL with debug information<td>MEDIA_TYPE_COMMAND<td>MEDIA_SUBTYPE_COMMAND_GCL
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
* <tr><td>Path<td>src/algorithms/AADC_LaneDetection
* <tr><td>Filename<td>aadc_LaneDetection.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/

//!  Template filter for OpenCV Image Processing
/*!
* This class is the main class of the OpenCV Template Filter and can be used as template for user specific image processing filters
*/
class LaneSteeringBeta : public adtf::cFilter
{

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );
protected:
    /*! input for rgb image */
    cVideoPin           m_oVideoInputPin;

    /*! Output for rgb image */
    cVideoPin           m_oVideoOutputPin;



    /*! output for GCL */
    cOutputPin           m_oGCLOutputPin;

    /*! output fot steering angle */
    cOutputPin m_oOutputSteeringAngle;
    cObjectPtr<IMediaDescriptionManager> pDescManager;


    cCriticalSection CritSteeringOut;
    cCriticalSection CritDynWidthIn;


public:
    cCriticalSection m_critSecMinimumUsValue;
    /*! Media Type Descriptions */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValueTwo;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValueThree;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolSignalValue;

    tBufferID	m_szIDOutputSteeringValue;
    tBufferID	m_szIDOutputSteeringTimeStamp;
    tBool		m_szIDsOutputSteeringSet;
    tUInt32 timeofframe;
    tFloat32 cumulatedDifference;
    tFloat32 SteeringValue;
    tFloat32 LastSteeringValue;

    tInt32 dynWidthR;
    tInt32 dynWidthL;

    cInputPin ResetDynamicWidthIn;
	tBufferID BoolID;
	tBufferID BoolIDTs;
	tBool ResetSet;
	tBool ResetSteering;
	tBool Reverse;


	cInputPin SwitchLaneIn;
	tBufferID SwitchLaneBuffer;
	tBufferID SwitchLaneBufferTs;
	tBool	  SwitchLaneIDSet;
    cCriticalSection CritBoolIn;

	cInputPin WobbleIn;
	tBufferID WobbleBuffer;
	tBufferID WobbleBufferTs;
	tBool	  WobbleIDSet;
    cCriticalSection CritWobbleIn;

    cInputPin DistanceIn;
    tBufferID DistanceBuffer;
    tBufferID DistanceBufferTs;
    tBool	  DistanceBufferIDSet;
    cCriticalSection CritDistanceIn;

    tBool OverallDistanceSet;
    tBool LaneSwitchInProcess;
    tFloat32 DistanceAtStart;
    tFloat32 DistanceDriven;
    tBool RightLane;
    tBool SwitchToRightLane;
    tBool SwitchToLeftLane;
    tBool ROISwitched;
    tFloat32 LastSteering;


    tBool DistanceWobbleSet;
    tBool Wobbling;
    tFloat32 DistanceAtWobbleStart;
    tFloat32 DistanceDrivenWobble;
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    LaneSteeringBeta(const tChar* __info);


    /*! default destructor */
    virtual ~LaneSteeringBeta();

    /*! Implements the default cFilter state machine call. It will be
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
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

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

    /*! This Function is always called when any property has changed. This should be the only place
    *    to read from the properties itself and store their values in a member.
    *
    *    \param [in] strName the name of the property that has changed.
    *    \
    *    \return   Returns a standard result code.
    */
    tResult PropertyChanged(const tChar* strName);

private: // private methods

    /*!
     * Searches for the lanes.
     *
     * \param           detectionLines      The detection lines.
     * \param           image               The image.
     * \param [in,out]  detectedLinePoints  The left lane pixels.
     * \return  Returns a standard result code.
     *
     */
    tResult findLinePointsR(const vector<tInt>& detectionLines, const cv::Mat& image, vector <cPoint>& detectedLinePoints, tTimeStamp inputTimestamp);
    tResult findLinePointsL(const vector<tInt>& detectionLines, const cv::Mat& image, vector <cPoint>& detectedLinePoints, tTimeStamp inputTimestamp);

    /*!
     * Gets detection lines.
     *
     * \param [in,out]  detectionLines  The detection lines.
     *
     * \return  The detection lines.
     */
    tResult getDetectionLinesR(vector<tInt>& detectionLines);
    tResult getDetectionLinesL(vector<tInt>& detectionLines);


    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    *   \param pFormat the new format for the input pin
    *   \return Standard Result Code.
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! function to set the output image format
    *   \param outputImage the new format for the input pin
    *   \return Standard Result Code.
    */
    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);


    /*! function to process the mediasample
    *   \param pSample the new media sample
    *   \return Standard Result Code.
    */
    tResult ProcessVideo(IMediaSample* pSample, tTimeStamp inputTimestamp);
    tResult ProcessReset(IMediaSample* pSample);
    tResult ProcessSwitchLane(IMediaSample* pSample);
    tResult ProcessDistance(IMediaSample* pSample);
    tResult ProcessWobble(IMediaSample* pSample);

    /*! function to transmit steering angle */
    tResult TransmitSteering(tFloat32 steeringangle, tTimeStamp timestamp);

    /*! bitmap format of input pin */
    tBitmapFormat m_sInputFormat;

    /*! bitmap format of output pin */
    tBitmapFormat m_sOutputFormat;

    /*! tha last received input image*/
    Mat m_inputImage;

    /*! the struct with all the properties*/
    struct filterProperties
    {
        /*! Offset of the ROI in the Stream*/
        tInt ROIOffsetRX;
        /*! Offset of the ROI in the Stream*/
        tInt ROIOffsetRY;
        /*! Width of the ROI*/
        tInt ROIWidthR;
        /*! Height of the ROI*/
        tInt ROIHeightR;
        /*! number of detection lines searched in ROI */
        tInt detectionLines;
        /*! Minimum Line Width in Pixel */
        tInt minLineWidth;
        /*! Maximum Line Width in Pixel */
        tInt maxLineWidth;
        /*! Minimum line contrast in gray Values */
        tInt minLineContrast;
        /*! Threshold for image binarization */
        tInt thresholdImageBinarization;
        tInt ROIOffsetLX;
        /*! Offset of the ROI in the Stream*/
        tInt ROIOffsetLY;
        /*! Width of the ROI*/
        tInt ROIWidthL;
        /*! Height of the ROI*/
        tInt ROIHeightL;
        /*! number of detection lines searched in ROI */
        /*! x coordinate of upper reference linepoint */
        tFloat32 RefeRXU;
        /*! y coordinate of upper reference linepoint */
        tFloat32 RefeRYU;
        /*! x coordinate of lower reference linepoint */
        tFloat32 RefeRXL;
        /*! y coordinate of lower reference linepoint */
        tFloat32 RefeRYL;
        /*! x coordinate of upper reference linepoint */
        tFloat32 RefeLXU;
        /*! y coordinate of upper reference linepoint */
        tFloat32 RefeLYU;
        /*! x coordinate of lower reference linepoint */
        tFloat32 RefeLXL;
        /*! y coordinate of lower reference linepoint */
        tFloat32 RefeLYL;
        /*! Upper Bound for abs of cumulated Difference */
        tFloat32 UpperBound;
        /*! Steering Ouptut scale multiplier */
        tFloat32 SteeringMultiplier;
        /*! Paramaters for Canny */
        tFloat32 Threshold;
        tFloat32 ThresholdRatio;
        tFloat32 KernelSize;
        /*! Paramaters for Hough-Transform */
        tFloat32 AccumulatorThreshold;
        tInt32 maxDynamicWidthR;
        tInt32 maxDynamicWidthL;
        tInt32 dynWidthMultiplier;
        tFloat32 SteeringMultiplierR;
        tFloat32 SteeringOffset;
		tBool debugOutput;

		tFloat32 LaneSwitchDistanceLR;
		tFloat32 LaneSwitchOffsetLR;
		tInt	 LaneSwitchROIOffsetLR;
		tFloat32 LaneSwitchDistanceRL;
		tFloat32 LaneSwitchOffsetRL;
		tInt	 LaneSwitchROIOffsetRL;

    }
    /*! the filter properties of this class */
    m_filterProperties;

    /*!
     * Transmit gcl.
     *
     * \param   detectionLines  The detection lines.
     * \param   detectedLinePoints  The left lane pixels.
     *
     * \return  A tResult.
     */
    tResult transmitGCL(const vector<tInt>& detectionLines, const vector<cPoint>& detectedLinePoints, const vector<cPoint>& referencePoints);


tBool debugOutput;
};

/** @} */ // end of group

#endif  //_OPENCVTEMPLATE_FILTER_HEADER_
