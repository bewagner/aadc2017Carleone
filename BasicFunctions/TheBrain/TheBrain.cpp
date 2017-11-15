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
 * $Author:: spiesra $  $Date:: 2017-04-26 14:47:19#$ $Rev:: 62581   $
 **********************************************************************/
#include "functions.h"
#include "stdafx.h"
#include "TheBrain.h"
#include <exception>
#include <iostream>
#include <unistd.h>


/// Create filter shell
ADTF_FILTER_PLUGIN("TheBrain", OID_ADTF_TheBrain, TheBrain);


TheBrain::TheBrain(const tChar* __info):cFilter(__info)
{
	SetPropertyBool("DEBUG::DebutOutput", tFalse); 
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?"); 
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	m_SteeringInSet = tFalse;
	m_AIReadyBufferSet = tFalse;
	m_PersonTowardsStreetBufferSet = tFalse;
	m_CrossingClassificationBufferSet = tFalse;
	m_KidDetectedBufferSet = tFalse;
	m_ManueverActionBufferSet = tFalse;
	m_StopLineBufferSet = tFalse;
	m_LeftAllowedBufferSet = tFalse;
	m_RightAllowedBufferSet = tFalse;
	m_CrossingFreeBufferSet = tFalse;
	m_ManeuverFinishedBufferSet = tFalse;
	m_CarstateBufferSet = tFalse;
	m_ReadyBufferSet = tFalse;
	m_RunningBufferSet = tFalse;
	m_CompleteBufferSet = tFalse;
	m_RequestNewManeuverOutSet = tFalse;
	m_IndicatorLeftSet = tFalse;
	m_IndicatorRightSet = tFalse;
	m_IgnoreInBufferSet = tFalse;
	m_IgnoreOutBufferSet = tFalse;
	m_IgnoreTwoInBufferSet = tFalse;
	m_IgnoreTwoOutBufferSet = tFalse;
	m_IgnoreThreeInBufferSet = tFalse;
	m_IgnoreThreeOutBufferSet = tFalse;
	m_StartBufferSet = tFalse;
	m_ParkingSignBufferSet = tFalse;
	m_LookForCrossingBufferSet = tFalse;
	m_ZebraSignBufferSet = tFalse;
	m_ZebraLineBufferSet = tFalse;
	m_IgnoreFourInBufferSet = tFalse;
	m_ObsInBufferSet = tFalse;
	m_IgnoreZebraInSet = tFalse;
	m_IgnoreZebraOutSet = tFalse;
	m_ParkingBoolOutBufferSet = tFalse;

	//  TODO Set aIReady to true when we start working with the full configuration
	aIReady =  tFalse;
	kidDetected =  tFalse;
	personWalkingTowardsStreet =  tFalse;
	currentCrossingState = 0;
	isStarted = tFalse; 
	calibCounter = 0;
	readySend = tFalse;
	CrossingDetected = tFalse;
	LeftAllowed = tFalse;
	RightAllowed = tFalse;
	StoplineBool = tFalse;
	CrossingFree = tFalse;
	CurrentManeuver = -1;
	DriveToCrossingInProcess = tFalse;
	TurnInProcess = tFalse;
	TurnOutRightInProcess = tFalse;
	TurnOutLeftInProcess = tFalse;
	IgnoreCrossroads = tTrue;
	LookForCrossings = tFalse;
	IgnoreLookForCrossing = tFalse;
	WaitOnCrossing = tFalse;
	IgnoreSlowly = tFalse;
	FirstPositionReceived = tFalse;
	ParkingMode = tFalse;
	ParkingInProcess = tFalse;
	Complete = tFalse;
	DriveSlowlyTransmitted = tFalse;
	DriveToParkingSpace = tFalse;
	IgnoreTwo = tFalse;
	IgnoreThree = tFalse;
	ZebraSignBool = tFalse;
	ZebraLineBool = tFalse;
	WaitOnZebra = tFalse;
	ZebraIgnoreSent = tFalse;
	IgnoreFourSend = tFalse;
	IgnoreFourTransmit = tFalse;
	OvertakeInProcess = tFalse;

    CarstateOneSent = tFalse;
	DriveToZebrastreifen = tFalse;
	m_Counter = 0;
	m_ZebraCounter =0;


    SetPropertyFloat("DISTANCES::DistanceToCrossingLeft", 0.1f);
    SetPropertyStr("DISTANCES::DistanceToCrossingLeft" NSSUBPROP_DESCRIPTION, "Distance To Crossing Left");
    SetPropertyBool("DISTANCES::DistanceToCrossingLeft" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("DISTANCES::DistanceToCrossingRight", 0.1f);
    SetPropertyStr("DISTANCES::DistanceToCrossingRight" NSSUBPROP_DESCRIPTION, "Distance To Crossing Right");
    SetPropertyBool("DISTANCES::DistanceToCrossingRight" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("DISTANCES::DistanceToCrossingStopline", 0.1f);
    SetPropertyStr("DISTANCES::DistanceToCrossingStopline" NSSUBPROP_DESCRIPTION, "Distance To Crossing Stopline");
    SetPropertyBool("DISTANCES::DistanceToCrossingStopline" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("DISTANCES::DistanceToCrossing", 1.0f);
    SetPropertyStr("DISTANCES::DistanceToCrossing" NSSUBPROP_DESCRIPTION, "Distance To Crossing");
    SetPropertyBool("DISTANCES::DistanceToCrossing" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("DISTANCES::DistanceToCrossing", 1.0f);
	SetPropertyStr("DISTANCES::DistanceToCrossing" NSSUBPROP_DESCRIPTION, "Distance To Crossing");
	SetPropertyBool("DISTANCES::DistanceToCrossing" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("CLASSIFICATION::QueueLength", 3);
	SetPropertyStr("CLASSIFICATION::QueueLength" NSSUBPROP_DESCRIPTION, "Length of crossing classification queue.");
	SetPropertyBool("CLASSIFICATION::QueueLength" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("STEERING::minSteering", 20);
	SetPropertyStr("STEERING::minSteering" NSSUBPROP_DESCRIPTION, "SteeringValue of driving straight.");
	SetPropertyBool("STEERING::minSteering" NSSUBPROP_ISCHANGEABLE, tTrue);


}

TheBrain::~TheBrain()
{

}

tResult TheBrain::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		// get description for tInt32
		tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBool);
		cObjectPtr<IMediaType> pTypeSignalBool = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBool, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		tChar const * strDescInt32SignalValue = pDescManager->GetMediaDescription("tInt32SignalValue");
		RETURN_IF_POINTER_NULL(strDescInt32SignalValue);
		cObjectPtr<IMediaType> pTypeInt32SignalValue = new cMediaType(0, 0, 0, "tInt32SignalValue", strDescInt32SignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		RETURN_IF_FAILED(m_ManeuverActionInput.Create("Maneuver_Action", pTypeInt32SignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ManeuverActionInput));
		RETURN_IF_FAILED(pTypeInt32SignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalInt));

		RETURN_IF_FAILED(m_StopLineInput.Create("StopLineIn", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_StopLineInput));
		RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalBool));

		RETURN_IF_FAILED(m_SteeringIn.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_SteeringIn));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

		RETURN_IF_FAILED(m_VorfahrtInput.Create("Vorfahrt", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_VorfahrtInput));

		RETURN_IF_FAILED(m_ZebraSignInput.Create("ZebraSign", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ZebraSignInput));

		RETURN_IF_FAILED(m_ZebraLineInput.Create("ZebraLine", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ZebraLineInput));

		RETURN_IF_FAILED(m_IgnoreZebraIn.Create("IgnoreZebraIn", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreZebraIn));

		RETURN_IF_FAILED(m_IgnoreZebraOut.Create("IgnoreZebraOut", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreZebraOut));

		RETURN_IF_FAILED(m_RechtsVorLinksInput.Create("RechtsVorLinks", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_RechtsVorLinksInput));

		RETURN_IF_FAILED(m_AIReadyInput.Create("AIReady", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_AIReadyInput));

		RETURN_IF_FAILED(m_PersonTowardsStreet.Create("PersonTowardsStreet", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_PersonTowardsStreet));

		RETURN_IF_FAILED(m_CrossingClassificationInput.Create("CrossingClassification", pTypeInt32SignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_CrossingClassificationInput));

		RETURN_IF_FAILED(m_KidDetectedInput.Create("KidDetected", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_KidDetectedInput));

		RETURN_IF_FAILED(m_LeftAllowedInput.Create("LeftAllowed", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_LeftAllowedInput));

		RETURN_IF_FAILED(m_ObsIn.Create("ObsIn", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ObsIn));

		RETURN_IF_FAILED(m_RightAllowedInput.Create("RightAllowed", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_RightAllowedInput));

		RETURN_IF_FAILED(m_CrossingFreeInput.Create("CrossingFree", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_CrossingFreeInput));

		RETURN_IF_FAILED(m_ManeuverFinishedInput.Create("ManeuverFinished", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ManeuverFinishedInput));

		RETURN_IF_FAILED(m_ParkingSignInput.Create("ParkingSign", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ParkingSignInput));

		RETURN_IF_FAILED(m_CarstateOut.Create("Carstate", pTypeInt32SignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_CarstateOut));

		RETURN_IF_FAILED(m_ReadyOut.Create("StateReady", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ReadyOut));

		RETURN_IF_FAILED(m_RequestNewManeuverOut.Create("RequestNewManeuver", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_RequestNewManeuverOut));

		RETURN_IF_FAILED(m_IndicatorLeft.Create("IndicatorLeft", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IndicatorLeft));

		RETURN_IF_FAILED(m_IndicatorRight.Create("IndicatorRight", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IndicatorRight));

		RETURN_IF_FAILED(m_RunningOut.Create("StateRunning", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_RunningOut));

		RETURN_IF_FAILED(m_CompleteOut.Create("StateComplete", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_CompleteOut));

		RETURN_IF_FAILED(m_LookForCrossing.Create("LookForCrossing", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_LookForCrossing));

		RETURN_IF_FAILED(m_IgnoreOutput.Create("IgnoreOut", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreOutput));

		RETURN_IF_FAILED(m_StartInput.Create("Start", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_StartInput));

		RETURN_IF_FAILED(m_IgnoreInput.Create("IgnoreIn", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreInput));

		RETURN_IF_FAILED(m_IgnoreTwoInput.Create("IgnoreTwoIn", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreTwoInput));

		RETURN_IF_FAILED(m_IgnoreThreeInput.Create("DriveSlowly", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreThreeInput));

		RETURN_IF_FAILED(m_IgnoreFourInput.Create("IgnoreFour", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreFourInput));

		RETURN_IF_FAILED(m_IgnoreTwoOutput.Create("IgnoreTwoOut", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreTwoOutput));

		RETURN_IF_FAILED(m_IgnoreThreeOutput.Create("IgnoreThreeOut", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_IgnoreThreeOutput));

		RETURN_IF_FAILED(m_ParkingBoolOut.Create("ParkingBoolOut", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ParkingBoolOut));

	}
	else if (eStage == StageNormal)
	{
		// In this stage you would do further initialisation and/or create your dynamic pins.
		// Please take a look at the demo_dynamicpin example for further reference.
	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.
	}

	RETURN_NOERROR;
}

tResult TheBrain::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	if (eStage == StageGraphReady)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageFirst)
	{
	}

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult TheBrain::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pSource == &m_ManeuverActionInput)
		{
			ProcessManeuverAction(pMediaSample);
			if (CurrentManeuver == -2)
			{
				TransmitStateComplete(tTrue);
				TransmitCarstate(0);
				isStarted = tFalse;
				Complete = tTrue;
			}
			else {
				Complete = tFalse;
			}
		}

		if (pSource == &m_StartInput)
		{
			if (!readySend) RETURN_NOERROR;

			ProcessStart(pMediaSample);
		}
		if (Complete) RETURN_NOERROR;

		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);


		// Check if we received an input telling us that the neuronal nets are loaded.
		// Only when these have been loaded, we can start driving.

		if(pSource == &m_AIReadyInput) {
			ProcessAIReady(pMediaSample);
		}

		else if (pSource == &m_IgnoreTwoInput)
		{
			FirstPositionReceived = tTrue;
		}

		if (!readySend && FirstPositionReceived && aIReady) {
			//TransmitRequestNewManeuver(tTrue);
			TransmitStateReady(tTrue);
			readySend = tTrue;
			//TransmitRequestNewManeuver(tTrue);
		}


		if (!isStarted) RETURN_NOERROR;

		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		/*
		 * Function matches maneuver name to certain id
		 * 0 	- left
		 * 1 	- right
		 * 2 	- straight
		 * 3 	- parallel_parking
		 * 4 	- cross_parking
		 * 5 	- pull_out_left
		 * 6	- pull_out_right
		 */


		if (pSource == &m_StopLineInput)
		{
			if (LookForCrossings)
			{
				ProcessStopLine(pMediaSample);
				ActOnCrossing(1);
			}
		}
		else if (pSource == &m_LeftAllowedInput)
		{
			if (LookForCrossings && (CurrentManeuver == 0 || CurrentManeuver == 2))
			{
				ProcessLeftAllowed(pMediaSample);
				ActOnCrossing(2);
			}
		}


		else if (pSource == &m_ZebraSignInput){
			ProcessZebraSign(pMediaSample);
			if (SteeringValue < m_filterProperties.minSteering && !DriveToZebrastreifen && !OvertakeInProcess && isStarted && !DriveToCrossingInProcess && !TurnInProcess && !ParkingInProcess && !DriveToParkingSpace && !WaitOnCrossing && !ParkingMode &&!TurnOutRightInProcess && !TurnOutLeftInProcess)
			{
				TransmitCarstate(8);
				DriveToZebrastreifen = tTrue;
				//TransmitIgnoreThree(tTrue);
				//IgnoreFourTransmit = tTrue;
			}
		} else if (pSource == &m_ZebraLineInput) {

//			PRINT("ZebraLineInput");
			if (ZebraIgnoreSent) {
				RETURN_NOERROR;
			}
//			PRINT1("SteeringValue %f", SteeringValue);
//			if(!IgnoreFourTransmit) PRINT1("IgnoreFourTransmit %i", !IgnoreFourTransmit);
//			if(!OvertakeInProcess) PRINT1("OvertakeInProcess %i", !OvertakeInProcess);
//			if(!DriveSlowlyTransmitted) PRINT1("DriveSlowlyTransmitted %i", !DriveSlowlyTransmitted);
//			if(!isStarted) PRINT1("isStarted %i", !isStarted);
//			if(!DriveToCrossingInProcess) PRINT1("DriveToCrossingInProcess %i", !DriveToCrossingInProcess);
//			if(!TurnInProcess) PRINT1("TurnInProcess %i", !TurnInProcess);
//			if(!ParkingInProcess) PRINT1("ParkingInProcess %i", !ParkingInProcess);
//			if(!DriveToParkingSpace) PRINT1("DriveToParkingSpace %i", !DriveToParkingSpace);
//			if(!WaitOnCrossing) PRINT1("WaitOnCrossing %i", !WaitOnCrossing);
//			if(!ParkingMode) PRINT1("ParkingMode %i", !ParkingMode);
//			if(!TurnOutRightInProcess) PRINT1("TurnOutRightInProcess %i", !TurnOutRightInProcess);
//			if(!TurnOutLeftInProcess) PRINT1("TurnOutLeftInProcess %i", !TurnOutLeftInProcess);
//			ProcessZebraLine(pMediaSample);
			if (SteeringValue < m_filterProperties.minSteering
					&& !IgnoreFourTransmit && !OvertakeInProcess
					&& !DriveSlowlyTransmitted && isStarted
					&& !DriveToCrossingInProcess && !TurnInProcess
					&& !ParkingInProcess && !DriveToParkingSpace
					&& !WaitOnCrossing && !ParkingMode && !TurnOutRightInProcess
					&& !TurnOutLeftInProcess) {
//				PRINT("Transmit Carstate 0")
				TransmitCarstate(0);
//				TransmitIgnoreThree(tTrue);
//				IgnoreFourTransmit = tFalse;
				WaitOnZebra = tTrue;
				TransmitIgnoreZebra(tTrue);
				ZebraIgnoreSent = tTrue;
			}
		}
		else if (pSource == &m_RightAllowedInput)
		{
			if (LookForCrossings && (CurrentManeuver == 1 || CurrentManeuver == 2))
			{
				ProcessRightAllowed(pMediaSample);
				ActOnCrossing(3);
			}
			if (ParkingMode)
			{
				if(m_filterProperties.debugOutput) { 
					LOG_INFO(cString("ActOnParkingSpot reached")); 
				}
				ActOnParkingSpot();
				ParkingMode = tFalse;
			}
		}
		else if (pSource == &m_CrossingFreeInput)
		{
			ProcessCrossingFree(pMediaSample);
		}
		else if (pSource == &m_ManeuverFinishedInput)
		{
			ProcessManeuverFinished(pMediaSample);
		}
		else if (pSource == &m_ObsIn)
		{
            if(OvertakeInProcess) PRINT1("OvertakeInProcess %i", !OvertakeInProcess);
            if(DriveToZebrastreifen) PRINT1("DriveToZebrastreifen %i", !DriveToZebrastreifen);
            if(DriveSlowlyTransmitted) PRINT1("DriveSlowlyTransmitted %i", !DriveSlowlyTransmitted);
            if(!isStarted) PRINT1("isStarted %i", !isStarted);
            if(DriveToCrossingInProcess) PRINT1("DriveToCrossingInProcess %i", !DriveToCrossingInProcess);
            if(TurnInProcess) PRINT1("TurnInProcess %i", !TurnInProcess);
            if(ParkingInProcess) PRINT1("ParkingInProcess %i", !ParkingInProcess);
            if(DriveToParkingSpace) PRINT1("DriveToParkingSpace %i", !DriveToParkingSpace);
            if(WaitOnCrossing) PRINT1("WaitOnCrossing %i", !WaitOnCrossing);
            if(ParkingMode) PRINT1("ParkingMode %i", !ParkingMode);
            if(TurnOutRightInProcess) PRINT1("TurnOutRightInProcess %i", !TurnOutRightInProcess);
            if(TurnOutLeftInProcess) PRINT1("TurnOutLeftInProcess %i", !TurnOutLeftInProcess);


			if (!DriveToZebrastreifen && !OvertakeInProcess && !DriveSlowlyTransmitted && isStarted && !DriveToCrossingInProcess && !TurnInProcess && !ParkingInProcess && !DriveToParkingSpace && !WaitOnCrossing && !ParkingMode && !TurnOutRightInProcess && !TurnOutLeftInProcess){
				//ProcessObs(pMediaSample);
				TransmitCarstate(15);
				OvertakeInProcess = tTrue;
			}
		}
		else if (pSource == &m_IgnoreInput)
		{
			ProcessIgnore(pMediaSample);
		}
		else if (pSource == &m_SteeringIn)
		{
			ProcessSteering(pMediaSample);
		}
		else if (pSource == &m_IgnoreFourInput)
		{
			if(!DriveToZebrastreifen && IgnoreFourTransmit && !OvertakeInProcess && !DriveSlowlyTransmitted && isStarted && !DriveToCrossingInProcess && !TurnInProcess && !ParkingInProcess && !DriveToParkingSpace && !WaitOnCrossing && !ParkingMode && !TurnOutRightInProcess && !TurnOutLeftInProcess) {

				TransmitCarstate(1);

			}
			ZebraSignBool = tFalse;
			IgnoreFourTransmit = tFalse;
			TransmitBrakeLight(tFalse);

		}
		else if (pSource == &m_IgnoreThreeInput)
		{
			if (!DriveSlowlyTransmitted && isStarted && !DriveToCrossingInProcess && !TurnInProcess && !ParkingInProcess && !DriveToParkingSpace && !TurnOutRightInProcess && !TurnOutLeftInProcess){
				TransmitCarstate(12);
				DriveSlowlyTransmitted = tTrue;
			}
		}
		else if (pSource == &m_LookForCrossing)
		{
			if (!IgnoreLookForCrossing) {
				ProcessLookForCrossing(pMediaSample);
				IgnoreLookForCrossing = tTrue;
				if(m_filterProperties.debugOutput) LOG_INFO(cString("LookingForCrossings"));
			}
		}
		else if (pSource == &m_ParkingSignInput)
		{
			ProcessParkingSign(pMediaSample);
			if (SeesParkingSign && CurrentManeuver == 4)
			{
				ParkingMode = tTrue;
				LookForCrossings = tFalse;
				IgnoreLookForCrossing = tTrue;
				DriveSlowlyTransmitted = tTrue;

            } else {
                //if(!CarstateOneSent) {
                //TransmitCarstate(1);
                //CarstateOneSent = tTrue;
                //}
            }
		}
		else if(pSource == &m_PersonTowardsStreet) {

			ProcessPersonTowardsStreet(pMediaSample);
		}
		else if(pSource == &m_IgnoreZebraIn)
		{
			ProcessIgnoreZebra(pMediaSample);
		}

		else if(pSource == &m_VorfahrtInput) {
			ProcessVorfahrt(pMediaSample);
		}
		else if(pSource == &m_RechtsVorLinksInput) {
			ProcessRechtsVorLinks(pMediaSample);
		}
		else if(pSource == &m_CrossingClassificationInput)
		{
			ProcessCrossingClassification(pMediaSample);
			//PRINT1("WaitOnZebra %i", WaitOnZebra)
                if (WaitOnZebra)
                {
                    PRINT("WaitOnZebraReached");
                    PRINT1("queue length: %i", m_filterProperties.CrossingClassificationQueueSize)
                        if(m_Counter++ < m_filterProperties.CrossingClassificationQueueSize){
                            //m_Counter++;
                            PRINT1("Brain counter: %i", m_Counter)
                                RETURN_NOERROR;
                        }
                    PRINT1("personWalkingTowardsStreet, %i", personWalkingTowardsStreet)
                        if(!personWalkingTowardsStreet)
                        {
                            TransmitCarstate(1);
                            WaitOnZebra = tFalse;
                            m_Counter = 0;
                        }
                }


			if(WaitOnCrossing)
			{
				// Wait till we saw at least the queue of classifications to make sure we are classifying right.
				if(m_Counter++ < m_filterProperties.CrossingClassificationQueueSize){
					RETURN_NOERROR;
				}

				carOnLeftSide = TrafficSituation[0];
				carInCenter = TrafficSituation[1];
				carOnRightSide = TrafficSituation[2];

				// If a person is walking towards the street, we will wait for it to be gone.
				if(personWalkingTowardsStreet) {
					if(m_filterProperties.debugOutput) {
						LOG_INFO(cString("Person walking towards street. We will wait for it to be gone and continue driving then."));
					}
					RETURN_NOERROR;
				}

				// Otherwise we check it there are any cars on the crossing and drive, if we can.
				switch (CurrentManeuver)
				{
                    // Links abbiegen
				case 0:
					if(RechtsVorLinks)
					{
						if(carOnRightSide || carInCenter)
						{
							break;
						}

						GoCrossing();
						break;
					}

					if(Vorfahrt)
					{
						if(carInCenter)
						{
							break;
						}

						GoCrossing();
						break;
					}
					// Die anderen haben Vorfahrt, daher warten wir auf alle.
					if(carOnLeftSide || carInCenter || carOnRightSide)
					{
						break;
					}

					GoCrossing();
					break;

					// Rechts abbiegen
				case 1:
					if(RechtsVorLinks || Vorfahrt)
					{
						GoCrossing();
						break;
					}

					if(carOnLeftSide)
					{
						break;
					}

					GoCrossing();
					break;
					// Geradeaus fahren
				case 2:
					if(RechtsVorLinks)
					{
						if(carOnRightSide)
						{
							break;
						}

						GoCrossing();
						break;

					}

					if(Vorfahrt)
					{
						GoCrossing();
						break;
					}

					if(carOnLeftSide || carOnRightSide)
					{
						break;
					}

					GoCrossing();
					break;

				default:
					break;
				}
			}
		}
		else if(pSource == &m_KidDetectedInput) {
			ProcessKidDetected(pMediaSample);
		}
	}
	RETURN_NOERROR;
}

tResult TheBrain::ProcessKidDetected(IMediaSample* pMediaSample)
{

	tTimeStamp ts;
	__synchronized_obj(CritKidDetected);
	{
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_KidDetectedBufferSet)
        {
            pCoderInput->GetID("bValue",m_KidDetectedBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_KidDetectedBufferIDTs);
            m_KidDetectedBufferSet = tTrue;
        }

        pCoderInput->Get(m_KidDetectedBufferID, (tVoid*)&kidDetected);
        pCoderInput->Get(m_KidDetectedBufferIDTs, (tVoid*)&ts);
    }
	if (!DriveToZebrastreifen && kidDetected && !DriveSlowlyTransmitted && isStarted && !DriveToCrossingInProcess && !TurnInProcess && !ParkingInProcess && !DriveToParkingSpace && !WaitOnCrossing && !ParkingMode &&!TurnOutRightInProcess && !TurnOutLeftInProcess && !WaitOnZebra && !WaitOnCrossing){
		TransmitCarstate(11);
	}
	RETURN_NOERROR;
}

tResult TheBrain::TransmitBrakeLight(tBool BoolVal)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    tTimeStamp ts = GetTime();
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputBrakeLightSet)
        {
            pCoderOutput->GetID("bValue",m_szIDOutputBrakeLightValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDOutputBrakeLightTimeStamp);
            m_szIDsOutputBrakeLightSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputBrakeLightValue, (tVoid*)&BoolVal);
        pCoderOutput->Set(m_szIDOutputBrakeLightTimeStamp, (tVoid*)&ts);

    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputBrakeLight.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult TheBrain::ProcessCrossingClassification(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritCrossingClassification);

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalInt, pMediaSample, pCoderInput);

        if(!m_CrossingClassificationBufferSet)
        {
            pCoderInput->GetID("intValue",m_CrossingClassificationBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_CrossingClassificationBufferIDTs);
            m_CrossingClassificationBufferSet = tTrue;

        }


		pCoderInput->Get(m_CrossingClassificationBufferID, (tVoid*)&currentCrossingState);
		pCoderInput->Get(m_CrossingClassificationBufferIDTs, (tVoid*)&ts);
	}

	// Jeweils letzten Eintrag löschen.
	leftCrossingClassification.pop_front();
	centerCrossingClassification.pop_front();
	rightCrossingClassification.pop_front();


	// Decodieren des Inputs
	// hunderter left
	// zehner center
	// einer right
	tBool left	= currentCrossingState > 99;
	tBool right	= (currentCrossingState % 10) > 0;
	tBool center  = (currentCrossingState % 100) > 9;

	// Und neue Einträge einfügen.
	leftCrossingClassification.push_back(left);
	centerCrossingClassification.push_back(center);
	rightCrossingClassification.push_back(right);

	tInt leftSum = 0;
	tInt centerSum = 0;
	tInt rightSum = 0;

	for(int i = 0; i < (int) leftCrossingClassification.size(); i++) {
		leftSum += leftCrossingClassification[i];
		centerSum += centerCrossingClassification[i];
		rightSum += rightCrossingClassification[i];
	}

	// Zurücksetzen des TrafficSituationVectors
	TrafficSituation.clear();

	tInt threshold = 1;

	TrafficSituation.push_back(leftSum > threshold);
	TrafficSituation.push_back(centerSum > threshold);
	TrafficSituation.push_back(rightSum > threshold);

    RETURN_NOERROR;
}




tResult TheBrain::ProcessPersonTowardsStreet(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritPersonTwoardsStreet);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_PersonTowardsStreetBufferSet)
        {
            pCoderInput->GetID("bValue",m_PersonTowardsStreetBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_PersonTowardsStreetBufferIDTs);
            m_PersonTowardsStreetBufferSet = tTrue;
        }

        pCoderInput->Get(m_PersonTowardsStreetBufferID, (tVoid*)&personWalkingTowardsStreet);
        pCoderInput->Get(m_PersonTowardsStreetBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessManeuverAction(IMediaSample* pMediaSample)
{

	tTimeStamp ts;
	__synchronized_obj(CritManeuverIn);
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalInt, pMediaSample, pCoderInput);

		if(!m_ManueverActionBufferSet)
		{
			pCoderInput->GetID("intValue",m_ManeuverActionBufferID);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_ManeuverActionBufferIDTs);
			m_ManueverActionBufferSet = tTrue;
		}

		pCoderInput->Get(m_ManeuverActionBufferID, (tVoid*)&CurrentManeuver);
		pCoderInput->Get(m_ManeuverActionBufferIDTs, (tVoid*)&ts);
		if(m_filterProperties.debugOutput) {
			LOG_INFO(cString::Format("Maneuver %i", CurrentManeuver));
		}
		}
	if( CurrentManeuver == 4) TransmitParkingBool(tTrue);
	else TransmitParkingBool(tFalse);

	RETURN_NOERROR;

}

tResult TheBrain::ProcessZebraLine(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritZebraLine);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_ZebraLineBufferSet)
        {
            pCoderInput->GetID("bValue",m_ZebraLineBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_ZebraLineBufferIDTs);
            m_ZebraLineBufferSet = tTrue;
        }

        pCoderInput->Get(m_ZebraLineBufferID, (tVoid*)&ZebraLineBool);
        pCoderInput->Get(m_ZebraLineBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessZebraSign(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritZebraSign);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_ZebraSignBufferSet)
        {
            pCoderInput->GetID("bValue",m_ZebraSignBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_ZebraSignBufferID);
            m_ZebraSignBufferSet = tTrue;
        }

        pCoderInput->Get(m_ZebraSignBufferID, (tVoid*)&ZebraSignBool);
        pCoderInput->Get(m_ZebraSignBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}


tResult TheBrain::ProcessStopLine(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_StopLineBufferSet)
        {
            pCoderInput->GetID("bValue",m_StopLineBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_StopLineBufferIDTs);
            m_StopLineBufferSet = tTrue;
        }

        pCoderInput->Get(m_StopLineBufferID, (tVoid*)&StoplineBool);
        pCoderInput->Get(m_StopLineBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessIgnoreZebra(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritZebraIgnoreIn);
	tTimeStamp ts;
	tBool bValue;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

		if(!m_IgnoreZebraInSet)
		{
			pCoderInput->GetID("bValue",m_IgnoreZebraInID);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_IgnoreZebraInIDTs);
			m_IgnoreZebraInSet = tTrue;
		}

		pCoderInput->Get(m_IgnoreZebraInID, (tVoid*)&bValue);
		pCoderInput->Get(m_IgnoreZebraInIDTs, (tVoid*)&ts);
		ZebraIgnoreSent = tFalse;
	}
	RETURN_NOERROR;
}


tResult TheBrain::ProcessLeftAllowed(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritLeftAllowed);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_LeftAllowedBufferSet)
        {
            pCoderInput->GetID("bValue",m_LeftAllowedBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_LeftAllowedBufferIDTs);
            m_LeftAllowedBufferSet = tTrue;
        }

        pCoderInput->Get(m_LeftAllowedBufferID, (tVoid*)&LeftAllowed);
        pCoderInput->Get(m_LeftAllowedBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessAIReady(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritAIReady);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_AIReadyBufferSet)
        {
            pCoderInput->GetID("bValue",m_AIReadyBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_AIReadyBufferIDTs);
            m_AIReadyBufferSet = tTrue;
        }

        pCoderInput->Get(m_AIReadyBufferID, (tVoid*)&aIReady);
        pCoderInput->Get(m_AIReadyBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}




tResult TheBrain::ProcessRightAllowed(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritRightAllowed);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_RightAllowedBufferSet)
        {
            pCoderInput->GetID("bValue",m_RightAllowedBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_RightAllowedBufferIDTs);
            m_RightAllowedBufferSet = tTrue;
        }

        pCoderInput->Get(m_RightAllowedBufferID, (tVoid*)&RightAllowed);
        pCoderInput->Get(m_RightAllowedBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}


tResult TheBrain::ProcessCrossingFree(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_CrossingFreeBufferSet)
        {
            pCoderInput->GetID("bValue",m_CrossingFreeBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_CrossingFreeBufferIDTs);
            m_CrossingFreeBufferSet = tTrue;
        }

        pCoderInput->Get(m_CrossingFreeBufferID, (tVoid*)&CrossingFree);
        pCoderInput->Get(m_CrossingFreeBufferIDTs, (tVoid*)&ts);
    }
    if (CrossingFree){
        GoCrossing();
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessStart(IMediaSample* pMediaSample)
{

	__synchronized_obj(CritStart);
	tBool value;
	tTimeStamp ts;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

		if(!m_StartBufferSet)
		{
			pCoderInput->GetID("bValue",m_StartBufferID);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_StartBufferIDTs);
			m_StartBufferSet = tTrue;
		}

		pCoderInput->Get(m_StartBufferID, (tVoid*)&value);
		pCoderInput->Get(m_StartBufferIDTs, (tVoid*)&ts);
	}
	if (value && !isStarted){
		DriveSlowlyTransmitted = tFalse;
		kidDetected =  tFalse;
		personWalkingTowardsStreet =  tFalse;
		currentCrossingState = 0;
		isStarted = tTrue;
		calibCounter = 0;
		readySend = tFalse;
		CrossingDetected = tFalse;
		LeftAllowed = tFalse;
		RightAllowed = tFalse;
		StoplineBool = tFalse;
		CrossingFree = tFalse;
		DriveToCrossingInProcess = tFalse;
		TurnInProcess = tFalse;
		TurnOutRightInProcess = tFalse;
		TurnOutLeftInProcess = tFalse;
		IgnoreCrossroads = tTrue;
		LookForCrossings = tFalse;
		IgnoreLookForCrossing = tFalse;
		WaitOnCrossing = tFalse;
		IgnoreSlowly = tFalse;
		ParkingMode = tFalse;
		Complete = tFalse;
		IgnoreFourSend = tFalse;
		ParkingInProcess = tFalse;
		TransmitStateComplete(tFalse);
		TransmitStateRunning(tTrue);
		ActOnManeuver(CurrentManeuver);
		OvertakeInProcess = tFalse;
		DriveToZebrastreifen = tFalse;
		WaitOnZebra = tFalse;
		ZebraIgnoreSent = tFalse;
	}
	else if (!value)
	{
		Complete = tFalse;
		TransmitCarstate(0);
		TransmitIndicatorLeft(tFalse);
		TransmitIndicatorRight(tFalse);
		TransmitIgnoreTwo(tFalse);
		isStarted = tFalse;

		aIReady =  tTrue;
		kidDetected =  tFalse;
		personWalkingTowardsStreet =  tTrue;
		currentCrossingState = 0;
		isStarted = tFalse;
		calibCounter = 0;
		readySend = tFalse;
		CrossingDetected = tFalse;
		LeftAllowed = tFalse;
		RightAllowed = tFalse;
		StoplineBool = tFalse;
		CrossingFree = tFalse;
		CurrentManeuver = -1;
		DriveToCrossingInProcess = tFalse;
		TurnInProcess = tFalse;
		TurnOutRightInProcess = tFalse;
		TurnOutLeftInProcess = tFalse;
		IgnoreCrossroads = tTrue;
		LookForCrossings = tFalse;
		IgnoreLookForCrossing = tTrue;
		WaitOnCrossing = tFalse;
		ParkingMode = tFalse;
		DriveToParkingSpace = tFalse;
		DriveSlowlyTransmitted = tTrue;
		IgnoreFourSend = tFalse;
		TransmitStateRunning(tFalse);
	}
	RETURN_NOERROR;

}

tResult TheBrain::ProcessParkingSign(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritParkingSign);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_ParkingSignBufferSet)
        {
            pCoderInput->GetID("bValue",m_ParkingSignInBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_ParkingSignInBufferIDTs);
            m_ParkingSignBufferSet = tTrue;
        }

        pCoderInput->Get(m_ParkingSignInBufferID, (tVoid*)&SeesParkingSign);
        pCoderInput->Get(m_ParkingSignInBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessVorfahrt(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritVorfahrt);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_VorfahrtBufferSet)
        {
            pCoderInput->GetID("bValue",m_VorfahrtBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_VorfahrtBufferIDTs);
            m_VorfahrtBufferSet = tTrue;
        }

        pCoderInput->Get(m_VorfahrtBufferID, (tVoid*)&Vorfahrt);
        pCoderInput->Get(m_VorfahrtBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessRechtsVorLinks(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritRechtsVorLinks);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_RechtsVorLinksBufferSet)
        {
            pCoderInput->GetID("bValue",m_RechtsVorLinksBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_RechtsVorLinksBufferIDTs);
            m_RechtsVorLinksBufferSet = tTrue;
        }

        pCoderInput->Get(m_RechtsVorLinksBufferID, (tVoid*)&RechtsVorLinks);
        pCoderInput->Get(m_RechtsVorLinksBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}


tResult TheBrain::ProcessIgnore(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritIgnoreIn);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_IgnoreInBufferSet)
        {
            pCoderInput->GetID("bValue",m_IgnoreInBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_IgnoreInBufferIDTs);
            m_IgnoreInBufferSet = tTrue;
        }

        pCoderInput->Get(m_IgnoreInBufferID, (tVoid*)&IgnoreCrossroads);
        pCoderInput->Get(m_IgnoreInBufferIDTs, (tVoid*)&ts);
    }
    //cout << "Ignore Value " << IgnoreCrossroads << " received" << endl;
    RETURN_NOERROR;
}

tResult TheBrain::ProcessIgnoreTwo(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    __synchronized_obj(CritIgnoreTwo);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_IgnoreTwoInBufferSet)
        {
            pCoderInput->GetID("bValue",m_IgnoreTwoInBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_IgnoreTwoInBufferIDTs);
            m_IgnoreTwoInBufferSet = tTrue;
        }

        pCoderInput->Get(m_IgnoreTwoInBufferID, (tVoid*)&IgnoreTwo);
        pCoderInput->Get(m_IgnoreTwoInBufferIDTs, (tVoid*)&ts);
    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessIgnoreThree(IMediaSample* pMediaSample)
{
    tTimeStamp ts;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_IgnoreThreeInBufferSet)
        {
            pCoderInput->GetID("bValue",m_IgnoreThreeInBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_IgnoreThreeInBufferIDTs);
            m_IgnoreThreeInBufferSet = tTrue;
        }

        pCoderInput->Get(m_IgnoreThreeInBufferID, (tVoid*)&IgnoreThree);
        pCoderInput->Get(m_IgnoreThreeInBufferIDTs, (tVoid*)&ts);
    }

    RETURN_NOERROR;

}
tResult TheBrain::ProcessIgnoreFour(IMediaSample* pMediaSample)
{
    __synchronized_obj(CritIgnoreFour);
    tTimeStamp ts;
    tBool IgnoreFour;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_IgnoreFourInBufferSet)
        {
            pCoderInput->GetID("bValue",m_IgnoreFourInBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_IgnoreFourInBufferIDTs);
            m_IgnoreFourInBufferSet = tTrue;
        }

        pCoderInput->Get(m_IgnoreFourInBufferID, (tVoid*)&IgnoreFour);
        pCoderInput->Get(m_IgnoreFourInBufferIDTs, (tVoid*)&ts);
    }

    RETURN_NOERROR;

}

tResult TheBrain::ProcessLookForCrossing(IMediaSample* pMediaSample)
{
    __synchronized_obj(CritLookForCrossing);
    tTimeStamp ts;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

        if(!m_LookForCrossingBufferSet)
        {
            pCoderInput->GetID("bValue",m_LookForCrossingBufferID);
            pCoderInput->GetID("ui32ArduinoTimestamp",m_LookForCrossingBufferIDTs);
            m_LookForCrossingBufferSet = tTrue;
        }

        pCoderInput->Get(m_LookForCrossingBufferID, (tVoid*)&LookForCrossings);
        pCoderInput->Get(m_LookForCrossingBufferIDTs, (tVoid*)&ts);

    }
    RETURN_NOERROR;
}

tResult TheBrain::ProcessManeuverFinished(IMediaSample* pMediaSample)
{

	tTimeStamp ts;
	__synchronized_obj(CritManeuverFinished);
	tBool ManeuverFinished;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

		if(!m_ManeuverFinishedBufferSet)
		{
			pCoderInput->GetID("bValue",m_ManeuverFinishedBufferID);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_ManeuverFinishedBufferIDTs);
			m_ManeuverFinishedBufferSet = tTrue;
		}

		pCoderInput->Get(m_ManeuverFinishedBufferID, (tVoid*)&ManeuverFinished);
		pCoderInput->Get(m_ManeuverFinishedBufferIDTs, (tVoid*)&ts);
	}
    
    // Hier setzten wir den Bool wieder auf false, damit beim nächsten Parkplatz wieder nur einmal CarState 1 ausgesendet wird.
    CarstateOneSent = tFalse;

	if (DriveToCrossingInProcess && !WaitOnCrossing)
	{
		//cout << "Finished DriveToCrossingInProcess" << endl;

		DriveToCrossingInProcess = tFalse;

		if(m_filterProperties.debugOutput) {
			LOG_INFO(cString("WaitOnCrossing"));
		}
		if(Vorfahrt && (CurrentManeuver == 1))
		{
			if(m_filterProperties.debugOutput) {
				LOG_INFO(cString("GoCrossing durch Vorfahrt"));
			}
			GoCrossing();
		}
		else {
			WaitOnCrossing = tTrue;
		}
	}
	else if (TurnInProcess)
	{
		if(m_filterProperties.debugOutput) {
			LOG_INFO(cString("Finished TurnInProcess"));
		}
		TransmitCarstate(1);
		TurnInProcess = tFalse;
		DriveToCrossingInProcess = tFalse;
		TransmitRequestNewManeuver(tTrue);
		TransmitIndicatorLeft(tFalse);
		TransmitIndicatorRight(tFalse);
		WaitOnCrossing = tFalse;
		IgnoreLookForCrossing = tFalse;
		DriveSlowlyTransmitted = tFalse;
	}
	if (TurnOutRightInProcess){
			LOG_INFO(cString("Finished TurnOutRightInProcess"));
		TurnOutRightInProcess = tFalse;
		TransmitIndicatorRight(tFalse);
		TransmitCarstate(1);
		TransmitRequestNewManeuver(tTrue);
		IgnoreLookForCrossing = tFalse;
		DriveSlowlyTransmitted = tFalse;
		TransmitIgnoreZebra(tTrue);
		ZebraIgnoreSent =tTrue;
	}
	if (TurnOutLeftInProcess)
	{
		if(m_filterProperties.debugOutput) {
			LOG_INFO(cString("Finished TurnOutLeftInProcess"));
		}
		TurnOutLeftInProcess = tFalse;
		TransmitIndicatorRight(tFalse);
		TransmitCarstate(1);
		TransmitRequestNewManeuver(tTrue);
		IgnoreLookForCrossing = tFalse;
		DriveSlowlyTransmitted = tFalse;
		TransmitIgnoreZebra(tTrue);
		ZebraIgnoreSent =tTrue;
	}

	if (ParkingInProcess)
	{
		ParkingMode = tFalse;
		if(m_filterProperties.debugOutput) {
			LOG_INFO(cString("Finished ParkingInProcess"));
		}
		TransmitCarstate(0);
		TransmitIndicatorRight(tFalse);
		TransmitRequestNewManeuver(tTrue);
		IgnoreLookForCrossing = tFalse;
		ParkingInProcess = tFalse;
		TransmitIgnoreTwo(tTrue);
		usleep(3000000);
		TransmitIgnoreTwo(tFalse);

		//DriveSlowlyTransmitted = tFalse;
		DriveToParkingSpace = tFalse;
		ActOnManeuver(CurrentManeuver);
		DriveSlowlyTransmitted = tFalse;
	}
	if (OvertakeInProcess)
	{
		OvertakeInProcess = tFalse;
	}
	if(DriveToZebrastreifen){
		DriveToZebrastreifen = tFalse;
		TransmitCarstate(0);
		WaitOnZebra = tTrue;
		IgnoreFourTransmit = tFalse;
	}
	RETURN_NOERROR;

}

tResult TheBrain::ProcessSteering(IMediaSample* pMediaSample) {
	tTimeStamp SteeringTimestamp;
	__synchronized_obj(CritSteeringIn);
	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

		if (!m_SteeringInSet) {
			pCoderInput->GetID("f32Value", m_SteeringInBufferID);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_SteeringInBufferIDTs);
			m_SteeringInSet = tTrue;
		}
		pCoderInput->Get(m_SteeringInBufferID, (tVoid*) &SteeringValue);
		pCoderInput->Get(m_SteeringInBufferIDTs, (tVoid*) &SteeringTimestamp);
	}
	RETURN_NOERROR;
}

tResult TheBrain::TransmitCarstate(tInt32 Carstate)
{
    __synchronized_obj(CritCarStateOut);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalInt->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalInt, pMediaSample, pCoderOutput);

        if (!m_CarstateBufferSet)
        {
            pCoderOutput->GetID("intValue",m_CarstateBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_CarstateBufferIDTs);
            m_CarstateBufferSet = tTrue;
        }
        pCoderOutput->Set(m_CarstateBufferID, (tVoid*)&Carstate);
        pCoderOutput->Set(m_CarstateBufferIDTs, (tVoid*)&time);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_CarstateOut.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult TheBrain::TransmitRequestNewManeuver(tBool Request)
{
    __synchronized_obj(CritRequestNewManeuver);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_RequestNewManeuverOutSet)
        {
            pCoderOutput->GetID("bValue",m_RequestNewManeuverOutBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_RequestNewManeuverOutBufferIDTs);
            m_RequestNewManeuverOutSet = tTrue;
        }
        pCoderOutput->Set(m_RequestNewManeuverOutBufferID, (tVoid*)&Request);
        pCoderOutput->Set(m_RequestNewManeuverOutBufferIDTs, (tVoid*)&time);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_RequestNewManeuverOut.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult TheBrain::TransmitIgnore(tBool value)
{
    __synchronized_obj(CritIgnoreOut);
    tTimeStamp ts = GetTime();
    IgnoreCrossroads = tTrue;
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_IgnoreOutBufferSet)
        {
            pCoderOutput->GetID("bValue",m_IgnoreOutBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_IgnoreOutBufferIDTs);
            m_IgnoreOutBufferSet = tTrue;
        }
        pCoderOutput->Set(m_IgnoreOutBufferID, (tVoid*)&value);
        pCoderOutput->Set(m_IgnoreOutBufferIDTs, (tVoid*)&ts);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_IgnoreOutput.Transmit(pMediaSample));


    RETURN_NOERROR;
}

tResult TheBrain::TransmitIgnoreTwo(tBool value)
{
    __synchronized_obj(CritIgnoreOutTwo);
    tTimeStamp ts = GetTime();
    IgnoreTwo = tTrue;
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_IgnoreTwoOutBufferSet)
        {
            pCoderOutput->GetID("bValue",m_IgnoreTwoOutBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_IgnoreTwoOutBufferIDTs);
            m_IgnoreTwoOutBufferSet = tTrue;
        }
        pCoderOutput->Set(m_IgnoreTwoOutBufferID, (tVoid*)&value);
        pCoderOutput->Set(m_IgnoreTwoOutBufferIDTs, (tVoid*)&ts);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_IgnoreTwoOutput.Transmit(pMediaSample));


    RETURN_NOERROR;
}

tResult TheBrain::TransmitIgnoreThree(tBool value)
{
    __synchronized_obj(CritIgnoreOutThree);
    tTimeStamp ts = GetTime();
    IgnoreThree = tTrue;
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_IgnoreThreeOutBufferSet)
        {
            pCoderOutput->GetID("bValue",m_IgnoreThreeOutBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_IgnoreThreeOutBufferIDTs);
            m_IgnoreThreeOutBufferSet = tTrue;
        }
        pCoderOutput->Set(m_IgnoreThreeOutBufferID, (tVoid*)&value);
        pCoderOutput->Set(m_IgnoreThreeOutBufferIDTs, (tVoid*)&ts);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_IgnoreThreeOutput.Transmit(pMediaSample));


    RETURN_NOERROR;
}


tResult TheBrain::TransmitIndicatorLeft(tBool Request)
{
    __synchronized_obj(CritIndicatorLeft);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_IndicatorLeftSet)
        {
            pCoderOutput->GetID("bValue",m_IndicatorLeftBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_IndicatorLeftBufferIDTs);
            m_IndicatorLeftSet = tTrue;
        }
        pCoderOutput->Set(m_IndicatorLeftBufferID, (tVoid*)&Request);
        pCoderOutput->Set(m_IndicatorLeftBufferIDTs, (tVoid*)&time);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_IndicatorLeft.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult TheBrain::TransmitIndicatorRight(tBool Request)
{
    __synchronized_obj(CritIndicatorRIght);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_IndicatorRightSet)
        {
            pCoderOutput->GetID("bValue",m_IndicatorRightBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_IndicatorRightBufferIDTs);
            m_IndicatorRightSet = tTrue;
        }
        pCoderOutput->Set(m_IndicatorRightBufferID, (tVoid*)&Request);
        pCoderOutput->Set(m_IndicatorRightBufferIDTs, (tVoid*)&time);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_IndicatorRight.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult TheBrain::TransmitIgnoreZebra(tBool Request)
{
	__synchronized_obj(CritZebraIgnoreOut);
	tUInt32 time = GetTime();
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_IgnoreZebraOutSet)
		{
			pCoderOutput->GetID("bValue",m_IgnoreZebraOutID);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_IgnoreZebraOutIDTs);
			m_IgnoreZebraOutSet = tTrue;
		}
		pCoderOutput->Set(m_IgnoreZebraOutID, (tVoid*)&Request);
		pCoderOutput->Set(m_IgnoreZebraOutIDTs, (tVoid*)&time);


		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_IgnoreZebraOut.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult TheBrain::TransmitStateReady(tBool StateReady)
{
    __synchronized_obj(CritStateReady);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_ReadyBufferSet)
        {
            pCoderOutput->GetID("bValue",m_ReadyBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_ReadyBufferIDTs);
            m_ReadyBufferSet = tTrue;
        }
        pCoderOutput->Set(m_ReadyBufferID, (tVoid*)&StateReady);
        pCoderOutput->Set(m_ReadyBufferIDTs, (tVoid*)&time);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_ReadyOut.Transmit(pMediaSample));

    RETURN_NOERROR;
}


tResult TheBrain::TransmitStateRunning(tBool StateRunning)
{
    __synchronized_obj(CritStateRunning);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_RunningBufferSet)
        {
            pCoderOutput->GetID("bValue",m_RunningBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_RunningBufferIDTs);
            m_RunningBufferSet = tTrue;
        }
        pCoderOutput->Set(m_RunningBufferID, (tVoid*)&StateRunning);
        pCoderOutput->Set(m_RunningBufferIDTs, (tVoid*)&time);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_RunningOut.Transmit(pMediaSample));

    RETURN_NOERROR;
}


tResult TheBrain::TransmitStateComplete(tBool StateComplete)
{
    __synchronized_obj(CritStateComplete);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_CompleteBufferSet)
        {
            pCoderOutput->GetID("bValue",m_CompleteBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_CompleteBufferIDTs);
            m_CompleteBufferSet = tTrue;
        }
        pCoderOutput->Set(m_CompleteBufferID, (tVoid*)&StateComplete);
        pCoderOutput->Set(m_CompleteBufferIDTs, (tVoid*)&time);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_CompleteOut.Transmit(pMediaSample));

    RETURN_NOERROR;
}


tResult TheBrain::TransmitParkingBool(tBool ParkingBool)
{
    __synchronized_obj(CritParkingBoolOut);
    tUInt32 time = GetTime();
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

        if (!m_ParkingBoolOutBufferSet)
        {
            pCoderOutput->GetID("bValue",m_ParkingBoolOutBufferID);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_ParkingBoolOutBufferIDTs);
            m_ParkingBoolOutBufferSet = tTrue;
        }
        pCoderOutput->Set(m_ParkingBoolOutBufferID, (tVoid*)&ParkingBool);
        pCoderOutput->Set(m_ParkingBoolOutBufferIDTs, (tVoid*)&time);


        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_ParkingBoolOut.Transmit(pMediaSample));

    RETURN_NOERROR;
}



tResult TheBrain::ActOnCrossing(tUInt32 Source)
{

	if(m_filterProperties.debugOutput) {
		LOG_INFO(cString("ActOnCrossing reached"));
	}
	LookForCrossings = tFalse;
	IgnoreFourSend = tFalse;
	if (!DriveToCrossingInProcess){
		DriveToCrossingInProcess = tTrue;

		if (CurrentManeuver == 0)
		{
			TransmitIndicatorLeft(tTrue);
		}
		else if (CurrentManeuver == 1 || CurrentManeuver == 4)
		{
			TransmitIndicatorRight(tTrue);
		}

		switch(Source){ //2=Left, 3=right, 1=stopline
		case 1:
			TransmitCarstate(8);
			break;
		case 2:
			TransmitCarstate(7);
			break;
		case 3:
			TransmitCarstate(6);
			break;
		}

	}
	RETURN_NOERROR;

}

tResult TheBrain::GoCrossing()
{

	IgnoreFourSend = tFalse;
	m_Counter = 0;
	//cout << "GoCrossing() reached." << endl;
	/*
	 * Function matches maneuver name to certain id
	 * 0 	- left
	 * 1 	- right
	 * 2 	- straight
	 * 3 	- parallel_parking
	 * 4 	- cross_parking
	 * 5 	- pull_out_left
	 * 6	- pull_out_right
	 */
	if(m_filterProperties.debugOutput) {
		LOG_INFO(cString("GoCrossing"));
	}
	WaitOnCrossing = tFalse;
	IgnoreLookForCrossing = tTrue;
	LookForCrossings = tFalse;
	//TransmitIgnore(tTrue);
	if (!TurnInProcess)
		TurnInProcess = tTrue;
	switch(CurrentManeuver){
	case 0:
		TransmitCarstate(4);
		break;
	case 1:
		TransmitCarstate(3);
		break;
	case 2:
		TransmitCarstate(2);
	}

	RETURN_NOERROR;

}

tResult TheBrain::ActOnManeuver(tInt Maneuver)
{

	if(m_filterProperties.debugOutput) {
		LOG_INFO(cString::Format("Act On Maneuver Reached with Maneuver %i", CurrentManeuver));
	}
	LookForCrossings = tFalse;
	IgnoreFourSend = tFalse;
	switch(Maneuver)
	{
	case 5: //TurnOutLeft
		TransmitCarstate(10);
		TransmitIndicatorLeft(tTrue);
		TurnOutLeftInProcess = tTrue;
		//TransmitIgnore(tTrue);
		break;
	case 6: //TurnOutRight
		TransmitCarstate(9);
		TransmitIndicatorRight(tTrue);
		TurnOutRightInProcess = tTrue;
		//TransmitIgnore(tTrue);
		break;
	default:
		TransmitCarstate(1);
		//IgnoreCrossroads = tFalse;
		break;
	}
	RETURN_NOERROR;

}

tResult TheBrain::ActOnParkingSpot()
{
    IgnoreFourSend = tFalse;
    DriveToParkingSpace = tTrue;
    TransmitIndicatorRight(tTrue);
    TransmitCarstate(13);
    ParkingMode = tFalse;
    ParkingInProcess = tTrue;

    RETURN_NOERROR;
}

tResult TheBrain::PropertyChanged(const tChar* strName)
{

	RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
	if (cString::IsEqual(strName, "DEBUG::DebutOutput"))
	{
		m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");
	}

	//associate the properties to the member
	if (cString::IsEqual(strName, "DISTANCES::DistanceToCrossingLeft"))
		m_filterProperties.DistanceCrossingLeft = GetPropertyFloat("DISTANCES::DistanceToCrossingLeft");
	else if (cString::IsEqual(strName, "DISTANCES::DistanceToCrossingRight"))
		m_filterProperties.DistanceCrossingRight = GetPropertyFloat("DISTANCES::DistanceToCrossingRight");
	else if (cString::IsEqual(strName, "DISTANCES::DistanceToCrossingStopline"))
		m_filterProperties.DistanceCrossingStopline = GetPropertyFloat("DISTANCES::DistanceToCrossingStopline");
	else if (cString::IsEqual(strName, "DISTANCES::DistanceToCrossing"))
		m_filterProperties.DistanceCrossing = GetPropertyFloat("DISTANCES::DistanceToCrossing");
	else if (cString::IsEqual(strName, "STEERING::minSteering"))
		m_filterProperties.minSteering = GetPropertyFloat("STEERING::minSteering");
	else if (cString::IsEqual(strName, "CLASSIFICATION::QueueLength")) {

		// // Anpassen an die neue Queue-Länge.
		int oldSize = leftCrossingClassification.size();
		m_filterProperties.CrossingClassificationQueueSize = GetPropertyFloat("CLASSIFICATION::QueueLength");
		int newSize = m_filterProperties.CrossingClassificationQueueSize;
		int queueSizeDifference =  abs(oldSize - newSize);

		for(int i = 0; i < queueSizeDifference; ++i) {
			if(newSize > oldSize) {
				leftCrossingClassification.push_back(tFalse);
				centerCrossingClassification.push_back(tFalse);
				rightCrossingClassification.push_back(tFalse);
			} else {
				leftCrossingClassification.pop_front();
				centerCrossingClassification.pop_front();
				rightCrossingClassification.pop_front();
			}
		}
	}

	RETURN_NOERROR;

}

tTimeStamp TheBrain::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}
