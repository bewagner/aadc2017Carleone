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
 * $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
 **********************************************************************/
#ifndef _DRIVE_DISTANCE_H_
#define _DRIVE_DISTANCE_H_

#define OID_ADTF_DRIVE_DISTANCE "adtf.user_driveDistance"


/*! @defgroup TemplateFilter
 *  @{
 *
 *  \image html User_Template.PNG "Plugin Template Filter"
 *
 * This is a small template which can be used by the AADC teams for their own filter implementations.
 * \b Dependencies \n
 * This plugin needs the following libraries:
 *
 *
 * <b> Filter Properties</b>
 * <table>
 * <tr><th>Property<th>Description<th>Default
 * </table>
 *
 * <b> Output Pins</b>
 * <table>
 * <tr><th>Pin<th>Description<th>MajorType<th>SubType
 * <tr><td>output_template<td>An example output pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
 *</table>
 *
 * <b> Input Pins</b>
 * <table>
 * <tr><th>Pin<th>Description<th>MajorType<th>SubType
 * <tr><td>input_template<td>An example input pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
 * </table>
 *
 * <b>Plugin Details</b>
 * <table>
 * <tr><td>Path<td>src/aadcUser/AADC_TemplateFilter
 * <tr><td>Filename<td>user_templateFilter.plb
 * <tr><td>Version<td>1.0.0
 * </table>
 *
 *
 */

//!  Template filter for AADC Teams
/*!
 * This is a example filter for the AADC
 */
class DriveDistance : public adtf::cFilter
{
	/*! set the filter ID and the version */
	ADTF_FILTER(OID_ADTF_DRIVE_DISTANCE, "Drive Distance", adtf::OBJCAT_DataFilter);

protected:
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue1;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue2;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue3;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue4;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue5;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolean;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionDriveDistance;



	cInputPin	m_oInputIgnoreBool; //tSignalValueBool
	tBufferID	m_szIdIgnoreBoolInValue;
	tBufferID	m_szIDIgnoreBoolTs;
	tBool		m_szIDInputIgnoreBoolSet;

	cInputPin	m_oInputIgnoreTwoBool; //tSignalValueBool
	tBufferID	m_szIdIgnoreTwoBoolInValue;
	tBufferID	m_szIDIgnoreTwoBoolTs;
	tBool		m_szIDInputIgnoreTwoBoolSet;

	cInputPin	m_oInputIgnoreThreeBool; //tSignalValueBool
	tBufferID	m_szIdIgnoreThreeBoolInValue;
	tBufferID	m_szIDIgnoreThreeBoolTs;
	tBool		m_szIDInputIgnoreThreeBoolSet;

	cInputPin	m_oInputBool; //tSignalValueBool
	tBufferID	m_szIdBoolInValue;
	tBufferID	m_szIDBoolTs;
	tBool		m_szIDInputBoolSet;

	cInputPin	m_oInputSpeed; //tSignalValueSpeed
	tBufferID	m_szIdSpeedInValue;
	tBufferID	m_szIDSpeedTs;
	tBool		m_szIDInputSpeedSet;

	cInputPin   m_oInputDistance;//tSignalValue
	tBufferID	m_szIdDistanceInValue;
	tBufferID	m_szIDDistanceTs;
	tBool		m_szIDInputDistanceSet;

	cInputPin	m_oInputDistanceOverall; //tSignalValue
	tBufferID	m_szIdDistanceOverall;
	tBufferID	m_szIDDistanceOverallTs;
	tBool		m_szIDDistanceOverallSet;

	cOutputPin  m_oOutputSpeedController; // tSignalValue
	tBufferID	m_szIDOutputSpeedControllerValue;
	tBufferID	m_szIDOutputSpeedControllerTimeStamp;
	tBool		m_szIDOutputSpeedSet;

	cOutputPin	m_oOutputBool; //tSignalValue
	tBufferID	m_szIdBoolOutValue;
	tBufferID	m_szIDBoolOutTs;
	tBool		m_szIDOutputBoolSet;

	cOutputPin	m_oOutputIgnoreBool; //tSignalValue
	tBufferID	m_szIdIgnoreBoolOutValue;
	tBufferID	m_szIDIgnoreBoolOutTs;
	tBool		m_szIDOutputIgnoreBoolSet;

	cOutputPin	m_oOutputIgnoreTwoBool; //tSignalValue
	tBufferID	m_szIdIgnoreTwoBoolOutValue;
	tBufferID	m_szIDIgnoreTwoBoolOutTs;
	tBool		m_szIDOutputIgnoreTwoBoolSet;

	cOutputPin	m_oOutputIgnoreThreeBool; //tSignalValue
	tBufferID	m_szIdIgnoreThreeBoolOutValue;
	tBufferID	m_szIDIgnoreThreeBoolOutTs;
	tBool		m_szIDOutputIgnoreThreeBoolSet;

	cOutputPin	m_oOutputBrakeLight; //tSignalValue
	tBufferID	m_szIdBrakeLightOutValue;
	tBufferID	m_szIDBrakeLightOutTs;
	tBool		m_szIDBrakeLightSet;

	vector<tFloat32> DistanceToDrive;
	tFloat32 DistanceToDriveSigned;
	tFloat32 DistanceDriven;
	tFloat32 DistanceDrivenIgnore;
	tFloat32 DistanceDrivenIgnoreTwo;
	tFloat32 DistanceDrivenIgnoreThree;
	tFloat32 DistanceAtStart;
	tFloat32 DistanceAtStartIgnore;
	tFloat32 DistanceAtStartIgnoreTwo;
	tFloat32 DistanceAtStartIgnoreThree;
	tFloat32 currentSpeed;
	tBool DriveInProcess;
	tBool OverallDistanceSet;
	tBool OverallDistanceIgnoreSet;
	tBool OverallDistanceIgnoreTwoSet;
	tBool OverallDistanceIgnoreThreeSet;
	tBool Reverse;
	tBool CurrentManoeuverFinished;
    tInt		ActiveManoeuverIndex;
	tUInt32 calibCounter;
	tBool FinishedTransmitted;
	tBool IgnoreTransmitted;
	tBool IgnoreTwoTransmitted;
	tBool IgnoreThreeTransmitted;
	tBool BlockTransmit;
	tBool Brake;

	cCriticalSection CritDistanceIn;
	cCriticalSection CritSpeedIn;
	cCriticalSection CritDistanceOverall;
	cCriticalSection CritBoolIn;
	cCriticalSection CritIgnoreIn;
	cCriticalSection CritIgnoreTwoIn;
	cCriticalSection CritIgnoreThreeIn;

	cCriticalSection CritSpeedOut;
	cCriticalSection CritBoolOut;
	cCriticalSection CritBrakeLightOut;
	cCriticalSection CritIgnoreOut;
	cCriticalSection CritIgnoreThreeOut;

public:
	/*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
	 */
	DriveDistance(const tChar* __info);

	/*! default destructor */
	virtual ~DriveDistance();

	struct filterProperties
	{
		tFloat32 defaultSpeed;
		tFloat32 brakeDistance;
		tFloat32 wheelDiameter;
		tFloat32 ignoreDistance;
		tFloat32 ignoreTwoDistance;
		tFloat32 ignoreThreeDistance;
	tBool debugOutput;
};
	/*! the filter properties of this class */
	filterProperties m_filterProperties;

protected:
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
	 *   \return Returns a standard result code.
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
	tResult ProcessDistanceToDrive(IMediaSample* pMediaSample);
	tResult ProcessDistanceOverall(IMediaSample* pMediaSample);
	tResult ProcessBool(IMediaSample* pMediaSample);
	tResult ProcessSpeed(IMediaSample* pMediaSample);
	tResult TransmitSpeed(tFloat32 speed, tUInt32 time);
	tResult TransmitBool(tBool finished, tUInt32 time);
	tResult TransmitBrakeLight(tBool status, tUInt32 time);
	tResult TransmitIgnore(tBool bvalue);
	tResult ProcessIgnore(IMediaSample* pMediaSample);
	tResult TransmitIgnoreTwo(tBool bvalue);
	tResult ProcessIgnoreTwo(IMediaSample* pMediaSample);
	tResult TransmitIgnoreThree(tBool bvalue);
	tResult ProcessIgnoreThree(IMediaSample* pMediaSample);
	tResult PropertyChanged(const tChar* strName);
	tUInt32 GetTime();
	tFloat32 Betrag(tFloat32 input);
};

//*************************************************************************************************
#endif // _TEMPLATE_FILTER_H_

/*!
 *@}
 */
