
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
   * $Author:: spiesra $  $Date:: 2017-05-09 08:51:06#$ $Rev:: 62951   $
   **********************************************************************/
#ifndef _TheBrain_
#define _TheBrain_

#define OID_ADTF_TheBrain "adtf.example.TheBrain"


#include <deque>


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
class TheBrain : public adtf::cFilter
{
	/*! set the filter ID and the version */
	ADTF_FILTER(OID_ADTF_TheBrain, "TheBrain", adtf::OBJCAT_DataFilter);

protected:

	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalInt;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalBool;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
	
	// INPUT PINS

	cInputPin 	m_AIReadyInput;
	tBufferID	m_AIReadyBufferID;
	tBufferID	m_AIReadyBufferIDTs;
	tBool		m_AIReadyBufferSet;

	cInputPin 	m_PersonTowardsStreet;
	tBufferID	m_PersonTowardsStreetBufferID;
	tBufferID	m_PersonTowardsStreetBufferIDTs;
	tBool		m_PersonTowardsStreetBufferSet;

	// This pin receives an int which classifies the situation we have at a crossing.
	cInputPin 	m_CrossingClassificationInput;
	tBufferID	m_CrossingClassificationBufferID;
	tBufferID	m_CrossingClassificationBufferIDTs;
	tBool		m_CrossingClassificationBufferSet;

	// When the AI stuff detects a child near the road,
	/* this pin receives a float which determines the distance to the child.  */
	cInputPin 	m_KidDetectedInput;
	tBufferID	m_KidDetectedBufferID;
	tBufferID	m_KidDetectedBufferIDTs;
	tBool		m_KidDetectedBufferSet;

	cInputPin 	m_ManeuverActionInput;
	tBufferID	m_ManeuverActionBufferID;
	tBufferID	m_ManeuverActionBufferIDTs;
	tBool		m_ManueverActionBufferSet;

	cInputPin 	m_StopLineInput;
	tBufferID	m_StopLineBufferID;
	tBufferID	m_StopLineBufferIDTs;
	tBool		m_StopLineBufferSet;

	cInputPin 	m_ZebraSignInput;
	tBufferID	m_ZebraSignBufferID;
	tBufferID	m_ZebraSignBufferIDTs;
	tBool		m_ZebraSignBufferSet;

	cInputPin 	m_ZebraLineInput;
	tBufferID	m_ZebraLineBufferID;
	tBufferID	m_ZebraLineBufferIDTs;
	tBool		m_ZebraLineBufferSet;

	cInputPin 	m_VorfahrtInput;
	tBufferID	m_VorfahrtBufferID;
	tBufferID	m_VorfahrtBufferIDTs;
	tBool		m_VorfahrtBufferSet;

	cInputPin 	m_RechtsVorLinksInput;
	tBufferID	m_RechtsVorLinksBufferID;
	tBufferID	m_RechtsVorLinksBufferIDTs;
	tBool		m_RechtsVorLinksBufferSet;

	cInputPin 	m_LeftAllowedInput;
	tBufferID	m_LeftAllowedBufferID;
	tBufferID	m_LeftAllowedBufferIDTs;
	tBool		m_LeftAllowedBufferSet;

	cInputPin 	m_RightAllowedInput;
	tBufferID	m_RightAllowedBufferID;
	tBufferID	m_RightAllowedBufferIDTs;
	tBool		m_RightAllowedBufferSet;

	cInputPin 	m_CrossingFreeInput;
	tBufferID	m_CrossingFreeBufferID;
	tBufferID	m_CrossingFreeBufferIDTs;
	tBool		m_CrossingFreeBufferSet;

	cInputPin 	m_ManeuverFinishedInput;
	tBufferID	m_ManeuverFinishedBufferID;
	tBufferID	m_ManeuverFinishedBufferIDTs;
	tBool		m_ManeuverFinishedBufferSet;

	cInputPin 	m_IgnoreInput;
	tBufferID	m_IgnoreInBufferID;
	tBufferID	m_IgnoreInBufferIDTs;
	tBool		m_IgnoreInBufferSet;

	cInputPin 	m_IgnoreTwoInput;
	tBufferID	m_IgnoreTwoInBufferID;
	tBufferID	m_IgnoreTwoInBufferIDTs;
	tBool		m_IgnoreTwoInBufferSet;

	cInputPin 	m_IgnoreThreeInput;
	tBufferID	m_IgnoreThreeInBufferID;
	tBufferID	m_IgnoreThreeInBufferIDTs;
	tBool		m_IgnoreThreeInBufferSet;

	cInputPin 	m_IgnoreFourInput;
	tBufferID	m_IgnoreFourInBufferID;
	tBufferID	m_IgnoreFourInBufferIDTs;
	tBool		m_IgnoreFourInBufferSet;

	cInputPin 	m_ParkingSignInput;
	tBufferID	m_ParkingSignInBufferID;
	tBufferID	m_ParkingSignInBufferIDTs;
	tBool		m_ParkingSignBufferSet;

	cOutputPin 	m_IgnoreOutput;
	tBufferID	m_IgnoreOutBufferID;
	tBufferID	m_IgnoreOutBufferIDTs;
	tBool		m_IgnoreOutBufferSet;

	cOutputPin 	m_IgnoreTwoOutput;
	tBufferID	m_IgnoreTwoOutBufferID;
	tBufferID	m_IgnoreTwoOutBufferIDTs;
	tBool		m_IgnoreTwoOutBufferSet;

	cOutputPin 	m_IgnoreThreeOutput;
	tBufferID	m_IgnoreThreeOutBufferID;
	tBufferID	m_IgnoreThreeOutBufferIDTs;
	tBool		m_IgnoreThreeOutBufferSet;

	cInputPin 	m_LookForCrossing;
	tBufferID	m_LookForCrossingBufferID;
	tBufferID	m_LookForCrossingBufferIDTs;
	tBool		m_LookForCrossingBufferSet;

	cInputPin 	m_StartInput;
	tBufferID	m_StartBufferID;
	tBufferID	m_StartBufferIDTs;
	tBool		m_StartBufferSet;

	cInputPin 	m_ObsIn;
	tBufferID	m_ObsInBufferID;
	tBufferID	m_ObsInBufferIDTs;
	tBool		m_ObsInBufferSet;

	cInputPin	m_SteeringIn; //tSignalValue
	tBufferID	m_SteeringInBufferID;
	tBufferID	m_SteeringInBufferIDTs;
	tBool		m_SteeringInSet;

	cOutputPin 	m_CarstateOut;
	tBufferID	m_CarstateBufferID;
	tBool		m_CarstateBufferSet;
	tBufferID	m_CarstateBufferIDTs;

	cOutputPin 	m_ReadyOut;
	tBufferID	m_ReadyBufferID;
	tBool		m_ReadyBufferSet;
	tBufferID	m_ReadyBufferIDTs;

	cOutputPin 	m_IgnoreZebraOut;
	tBufferID	m_IgnoreZebraOutID;
	tBool		m_IgnoreZebraOutSet;
	tBufferID	m_IgnoreZebraOutIDTs;

	cInputPin 	m_IgnoreZebraIn;
	tBufferID	m_IgnoreZebraInID;
	tBool		m_IgnoreZebraInSet;
	tBufferID	m_IgnoreZebraInIDTs;

	cOutputPin  m_oOutputBrakeLight; // tSignalValue
	tBufferID	m_szIDOutputBrakeLightValue;
	tBufferID	m_szIDOutputBrakeLightTimeStamp;
	tBool		m_szIDsOutputBrakeLightSet;

	cOutputPin 	m_RunningOut;
	tBufferID	m_RunningBufferID;
	tBool		m_RunningBufferSet;
	tBufferID	m_RunningBufferIDTs;

	cOutputPin 	m_CompleteOut;
	tBufferID	m_CompleteBufferID;
	tBool		m_CompleteBufferSet;
	tBufferID	m_CompleteBufferIDTs;

	cOutputPin 	m_RequestNewManeuverOut;
	tBufferID	m_RequestNewManeuverOutBufferID;
	tBool		m_RequestNewManeuverOutSet;
	tBufferID	m_RequestNewManeuverOutBufferIDTs;

	cOutputPin 	m_IndicatorLeft;
	tBufferID	m_IndicatorLeftBufferID;
	tBufferID	m_IndicatorLeftBufferIDTs;
	tBool		m_IndicatorLeftSet;

	cOutputPin 	m_IndicatorRight;
	tBufferID	m_IndicatorRightBufferID;
	tBufferID	m_IndicatorRightBufferIDTs;
	tBool		m_IndicatorRightSet;

	cOutputPin 	m_ParkingBoolOut;
	tBufferID	m_ParkingBoolOutBufferID;
	tBufferID	m_ParkingBoolOutBufferIDTs;
	tBool		m_ParkingBoolOutBufferSet;

	tBool		DriveToZebrastreifen;
	tBool		OvertakeInProcess;
	tBool       aIReady;
	tBool       kidDetected;
	tBool       personWalkingTowardsStreet;
	tBool		IgnoreFourTransmit;
	tInt32	 	currentCrossingState;
	tBool		readySend;
	tInt	 	calibCounter;
	tBool		LeftAllowed,RightAllowed;
	tBool		StoplineBool;
	tBool		CrossingFree;
	tInt		CurrentManeuver;
	tBool		CrossingDetected;
	tBool		DriveToCrossingInProcess;
	tBool		TurnInProcess;
	tBool		TurnOutRightInProcess;
	tBool		TurnOutLeftInProcess;
	tBool		isStarted;
	tBool		IgnoreCrossroads;
	tBool		SeesParkingSign;
	tBool		ParkingMode;
	tBool		IgnoreCrossingDueToParkingSign;
	tBool		IgnoreTwo;
	tBool		IgnoreThree;
	tBool		LookForCrossings;
	tBool		IgnoreLookForCrossing;
	tBool		Complete;
	tBool		IgnoreFourSend;

	tBool		RechtsVorLinks;
	tBool		Vorfahrt;
	vector<tBool>  TrafficSituation;
	tBool		WaitOnCrossing;
	tBool		DriveToParkingSpace;
	tBool		ParkingInProcess;
	tBool		DriveSlowlyTransmitted;
	tBool		IgnoreSlowly;
	tBool		FirstPositionReceived;
	tBool		ZebraSignBool;
	tBool		ZebraLineBool;	
    tBool  CarstateOneSent;
    

	tBool ZebraIgnoreSent;
	tBool carOnLeftSide ;
	tBool carInCenter ;
	tBool carOnRightSide;
	tInt  m_Counter;
	tInt  m_ZebraCounter;
	tBool WaitOnZebra;
	tFloat32 SteeringValue;

	cCriticalSection CritZebraIgnoreIn;
	cCriticalSection CritZebraIgnoreOut;
	cCriticalSection CritManeuverIn;
	cCriticalSection CritVorfahrt;
	cCriticalSection CritZebraSign;
	cCriticalSection CritZebraLine;
	cCriticalSection CritRechtsVorLinks;
	cCriticalSection CritAIReady;
	cCriticalSection CritPersonTwoardsStreet;
	cCriticalSection CritCrossingClassification;
	cCriticalSection CritKidDetected;
	cCriticalSection CritLeftAllowed;
	cCriticalSection CritRightAllowed;
	cCriticalSection CritManeuverFinished;
	cCriticalSection CritParkingSign;
	cCriticalSection CritLookForCrossing;
	cCriticalSection CritStart;
	cCriticalSection CritIgnoreIn;
	cCriticalSection CritIgnoreTwo;
	cCriticalSection CritDriveSLowly;
	cCriticalSection CritIgnoreFour;
	cCriticalSection CritSteeringIn;

	cCriticalSection CritCarStateOut;
	cCriticalSection CritStateReady;
	cCriticalSection CritRequestNewManeuver;
	cCriticalSection CritIndicatorLeft;
	cCriticalSection CritIndicatorRIght;
	cCriticalSection CritStateRunning;
	cCriticalSection CritStateComplete;
	cCriticalSection CritIgnoreOut;
	cCriticalSection CritIgnoreOutTwo;
	cCriticalSection CritIgnoreOutThree;
	cCriticalSection CritParkingBoolOut;



public:    
	/*! default constructor for template class
	  \param __info   [in] This is the name of the filter instance.
	*/
	TheBrain(const tChar* __info);

	/*! default destructor */
	virtual ~TheBrain();



protected:
	/*! Implements the default cFilter state machine call. It will be
	  called automatically by changing the filters state and needs
	  to be overwritten by the special filter.
	  Please see page_filter_life_cycle for further information on when the state of a filter changes.
	  \param [in,out] __exception   An Exception pointer where exceptions will be put when failed.
	  If not using the cException smart pointer, the interface has to
	  be released by calling Unref().
	  \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
	  \return Standard Result Code.
	*/
	tResult Init(tInitStage eStage, __exception);

	/*! Implements the default cFilter state machine call. It will be
	  called automatically by changing the filters state and needs
	  to be overwritten by the special filter.
	  Please see  page_filter_life_cycle for further information on when the state of a filter changes.
	  \param  eStage [in]    The Init function will be called when the filter state changes as follows:\n
	  \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
	  If not using the cException smart pointer, the interface has to
	  be released by calling Unref().
	  \return Standard Result Code.
	*/
	tResult Shutdown(tInitStage eStage, __exception);

	/*! This Function will be called by all pins the filter is registered to.
	  \param [in] pSource Pointer to the sending pin's IPin interface.
	  \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
	  \param [in] nParam1 Optional integer parameter.
	  \param [in] nParam2 Optional integer parameter.
	  \param [in] pMediaSample Address of an IMediaSample interface pointers.
                                                                        \
                                                                        \return   Returns a standard result code.
                                                                        \warning This function will not implement a thread-safe synchronization between the calls from different sources.
                                                                        You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
	*/
	tResult OnPinEvent(IPin* pSource,
					   tInt nEventCode,
					   tInt nParam1,
					   tInt nParam2,
					   IMediaSample* pMediaSample);
	tResult ProcessAIReady(IMediaSample* pMediaSample);
	tResult ProcessPersonTowardsStreet(IMediaSample* pMediaSample);
	tResult ProcessCrossingClassification(IMediaSample* pMediaSample);
	tResult ProcessKidDetected(IMediaSample* pMediaSample);
	tResult ProcessManeuverAction(IMediaSample* pMediaSample);
	tResult ProcessStopLine(IMediaSample* pMediaSample);
	tResult ProcessVorfahrt(IMediaSample* pMediaSample);
	tResult ProcessRechtsVorLinks(IMediaSample* pMediaSample);
	tResult ProcessLeftAllowed(IMediaSample* pMediaSample);
	tResult ProcessRightAllowed(IMediaSample* pMediaSample);
	tResult ProcessCrossingFree(IMediaSample* pMediaSample);
	tResult ProcessManeuverFinished(IMediaSample* pMediaSample);
	tResult ProcessParkingSign(IMediaSample* pMediaSample);
	tResult ProcessStart(IMediaSample* pMediaSample);
	tResult ProcessIgnore(IMediaSample* pMediaSample);
	tResult ProcessIgnoreTwo(IMediaSample* pMediaSample);
	tResult ProcessIgnoreThree(IMediaSample* pMediaSample);
	tResult ProcessIgnoreFour(IMediaSample* pMediaSample);
	tResult ProcessIgnoreZebra(IMediaSample* pMediaSample);
	tResult ProcessLookForCrossing(IMediaSample* pMediaSample);
	tResult ProcessObs(IMediaSample* pMediaSample);
	tResult TransmitCarstate(tInt32 Carstate);
	tResult TransmitRequestNewManeuver(tBool Request);
	tResult TransmitIndicatorLeft(tBool Request);
	tResult TransmitIndicatorRight(tBool Request);
	tResult TransmitStateReady(tBool StateReady);
	tResult TransmitStateRunning(tBool StateRunning);
	tResult TransmitStateComplete(tBool StateComplete);
	tResult TransmitResetDriveDistance(tBool ResetDistance);
	tResult TransmitDistance(tFloat32 distance, tUInt32 timestamp);
	tResult TransmitIgnore(tBool value);
	tResult TransmitIgnoreTwo(tBool value);
	tResult TransmitIgnoreThree(tBool value);
	tResult TransmitIgnoreZebra(tBool value);
	tResult TransmitBrakeLight(tBool boolVal);
	tResult TransmitParkingBool(tBool boolVal);
	tResult ActOnCrossing(tUInt32 Source);
	tResult ActOnManeuver(tInt Maneuver);
	tResult PropertyChanged(const tChar* strName);
	tResult GoCrossing();
	tResult ActOnParkingSpot();
	tResult ProcessZebraSign(IMediaSample* pMediaSample);
	tResult ProcessZebraLine(IMediaSample* pMediaSample);
	tResult ProcessSteering(IMediaSample* pMediaSample);

	

	std::deque<tBool> leftCrossingClassification;
	std::deque<tBool> centerCrossingClassification;
	std::deque<tBool> rightCrossingClassification;


	tTimeStamp GetTime();

	struct filterProperties
	{
		tFloat32 	DistanceCrossingLeft;
		tFloat32 	DistanceCrossingRight;
		tFloat32 	DistanceCrossingStopline;
		tFloat32	DistanceCrossing;
		tInt CrossingClassificationQueueSize;
		tBool debugOutput;
		tFloat32 	minSteering;
	};
	/*! the filter properties of this class */
	filterProperties m_filterProperties;
};

//*************************************************************************************************
#endif // _TEMPLATE_FILTER_H_

