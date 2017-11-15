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
#include "stdafx.h"
#include "driveDistance.h"
#include <iostream>
struct __exception;


/// Create filter shell
ADTF_FILTER_PLUGIN("Drive Distance", OID_ADTF_DRIVE_DISTANCE, DriveDistance);


DriveDistance::DriveDistance(const tChar* __info):cFilter(__info)
{
 SetPropertyBool("DEBUG::DebutOutput", tFalse); 
 SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?"); 
 SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyFloat("DefaultSpeed", -7.0f);
	SetPropertyStr("DefaultSpeed" NSSUBPROP_DESCRIPTION, "Obvious");
	SetPropertyBool("DefaultSpeed" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("BrakeDistance", 1.0f);
	SetPropertyStr("BrakeDistance" NSSUBPROP_DESCRIPTION, "Obvious");
	SetPropertyBool("BrakeDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("WheelDiameter", 0.34);
	SetPropertyStr("WheelDiameter" NSSUBPROP_DESCRIPTION, "Obvious");
	SetPropertyBool("WheelDiameter" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("IgnoreDistance", 1.5);
	SetPropertyStr("IgnoreDistance" NSSUBPROP_DESCRIPTION, "Obvious");
	SetPropertyBool("IgnoreDistance" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyFloat("IgnoreTwoDistance", 1.5);
	SetPropertyStr("IgnoreTwoDistance" NSSUBPROP_DESCRIPTION, "Obvious");
	SetPropertyBool("IgnoreTwoDistance" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyFloat("IgnoreThreeDistance", 1.5);
	SetPropertyStr("IgnoreThreeDistance" NSSUBPROP_DESCRIPTION, "Obvious");
	SetPropertyBool("IgnoreThreeDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	DistanceDriven = 0;
	DistanceDrivenIgnore = 0;
	DistanceDrivenIgnoreTwo = 0;
	DistanceToDriveSigned = 0;
	DistanceAtStart = 0;
	DistanceAtStartIgnoreTwo = 0;
	DistanceDrivenIgnoreThree = 0;
	DistanceAtStartIgnoreThree = 0;

	m_szIDBrakeLightSet = tFalse;
	m_szIDOutputSpeedSet = tFalse;
	m_szIDInputBoolSet = tFalse;
	m_szIDDistanceOverallSet = tFalse;
	m_szIDInputDistanceSet = tFalse;
	m_szIDOutputBoolSet = tFalse;
	m_szIDInputSpeedSet = tFalse;
	IgnoreTransmitted = tFalse;
	m_szIDOutputIgnoreBoolSet = tFalse;
	m_szIDInputIgnoreBoolSet = tFalse;
	m_szIDOutputIgnoreTwoBoolSet = tFalse;
	m_szIDInputIgnoreTwoBoolSet = tFalse;
	m_szIDOutputIgnoreThreeBoolSet = tFalse;
	m_szIDInputIgnoreThreeBoolSet = tFalse;

	CurrentManoeuverFinished = tFalse;
	DriveInProcess = tFalse;
	OverallDistanceSet = tFalse;
	OverallDistanceIgnoreSet = tTrue;
	OverallDistanceIgnoreTwoSet = tTrue;
	OverallDistanceIgnoreThreeSet = tTrue;
	calibCounter = 0;
	currentSpeed = -9.0f;
	ActiveManoeuverIndex = 0;
	FinishedTransmitted = tTrue;
	BlockTransmit = tFalse;
	IgnoreTwoTransmitted = tFalse;
	IgnoreThreeTransmitted = tTrue;
	Brake = tFalse;

}

DriveDistance::~DriveDistance()
{

}

tResult DriveDistance::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

		// in StageFirst you can create and register your static pins.
		if (eStage == StageFirst)
		{
			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
			//mediatype für signalvalues
			tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalValue);
			cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0,0,0,"tSignalValue",strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			//media type für bools
			tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(strDescBool);
			cObjectPtr<IMediaType> pTypeSignalBool = new cMediaType(0,0,0,"tBoolSignalValue",strDescBool, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

			tChar const * strDescDriveDistance = pDescManager->GetMediaDescription("driveDistanceCommand");
			RETURN_IF_POINTER_NULL(strDescDriveDistance);
			cObjectPtr<IMediaType> pTypeDriveDistance = new cMediaType(0,0,0,"driveDistanceCommand",strDescDriveDistance, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

			RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue1));
			RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue2));
			RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue3));
			RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue4));
			RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue5));
			RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean));
			RETURN_IF_FAILED(pTypeDriveDistance->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDriveDistance));


			RETURN_IF_FAILED(m_oInputDistance.Create("DistanceIn",pTypeDriveDistance,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputDistance));

			RETURN_IF_FAILED(m_oInputSpeed.Create("SpeedIn",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputSpeed));

			RETURN_IF_FAILED(m_oInputDistanceOverall.Create("DistanceOverall",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputDistanceOverall));

			RETURN_IF_FAILED(m_oInputBool.Create("BoolIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputBool));

			RETURN_IF_FAILED(m_oInputIgnoreBool.Create("IgnoreIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputIgnoreBool));

			RETURN_IF_FAILED(m_oInputIgnoreTwoBool.Create("IgnoreTwoIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputIgnoreTwoBool));

			RETURN_IF_FAILED(m_oInputIgnoreThreeBool.Create("IgnoreThreeIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputIgnoreThreeBool));

			RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedOut",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

			RETURN_IF_FAILED(m_oOutputBool.Create("BoolOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oOutputBool));

			RETURN_IF_FAILED(m_oOutputBrakeLight.Create("BrakeLightOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));
			RETURN_IF_FAILED(m_oOutputIgnoreBool.Create("IgnoreOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oOutputIgnoreBool));

			RETURN_IF_FAILED(m_oOutputIgnoreTwoBool.Create("IgnoreOutTwo",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oOutputIgnoreTwoBool));

			RETURN_IF_FAILED(m_oOutputIgnoreThreeBool.Create("IgnoreOutThree",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oOutputIgnoreThreeBool));
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

tResult DriveDistance::Shutdown(tInitStage eStage, __exception)
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

tResult DriveDistance::OnPinEvent(IPin* pSource,
								  tInt nEventCode,
								  tInt nParam1,
								  tInt nParam2,
								  IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (calibCounter <= 100){
			TransmitSpeed(0.0f,GetTime());
			calibCounter++;
		}
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		if (pSource == &m_oInputDistance)
		{
			ProcessDistanceToDrive(pMediaSample);
		}
		else if (pSource == &m_oInputSpeed)
		{
			ProcessSpeed(pMediaSample);
		}
		else if (pSource == &m_oInputBool)
		{
			ProcessBool(pMediaSample);
		}
		else if (pSource == &m_oInputIgnoreBool)
		{
			ProcessIgnore(pMediaSample);
		}
		else if (pSource == &m_oInputIgnoreTwoBool)
				{
					ProcessIgnoreTwo(pMediaSample);
				}
		else if (pSource == &m_oInputIgnoreThreeBool)
		{
			ProcessIgnoreThree(pMediaSample);
		}
		else if (pSource == &m_oInputDistanceOverall)
		{
			ProcessDistanceOverall(pMediaSample);
			if (!DistanceToDrive.empty() && OverallDistanceSet)
			{
				FinishedTransmitted = tFalse;

				tFloat32 speed = Reverse ? -currentSpeed: currentSpeed;

				if ((DistanceToDrive.at(ActiveManoeuverIndex) - DistanceDriven) > m_filterProperties.brakeDistance )
				{

					TransmitSpeed(speed, GetTime());
					//TransmitBrakeLight(tFalse, GetTime());
				}
				else if (((DistanceToDrive.at(ActiveManoeuverIndex) - DistanceDriven) <= m_filterProperties.brakeDistance) && ((DistanceToDrive.at(ActiveManoeuverIndex) - DistanceDriven) > 0))
				{
					tFloat32 lambda =  Brake ? (DistanceToDrive.at(ActiveManoeuverIndex) - DistanceDriven)/m_filterProperties.brakeDistance : 1.0f;
					speed *= lambda;
					if (Betrag(speed)< 0.2f)
					{
						speed = Reverse ? (tFloat32)-0.2 : (tFloat32)(0.2);
						//cout << "DD speed : "<< speed << endl;
					}
					TransmitSpeed(speed, GetTime());
					TransmitBrakeLight(tTrue, GetTime());
				}
				else if ((DistanceToDrive.at(ActiveManoeuverIndex) - DistanceDriven) < 0 )
				{
					OverallDistanceSet = tFalse;

					CurrentManoeuverFinished = tTrue;
					if ((tInt8) DistanceToDrive.size() >(tInt8) ActiveManoeuverIndex + 1)
					{
						ActiveManoeuverIndex++;
						CurrentManoeuverFinished = tFalse;
					}
					else
					{
						CurrentManoeuverFinished = tTrue;
						DistanceToDrive.clear();
						ActiveManoeuverIndex = 0;
						TransmitSpeed(0.0f, GetTime());
						TransmitBrakeLight(tTrue, GetTime());

					}

					//DriveInProcess = tFalse;

				}


			}
			else
			{
				TransmitBrakeLight(tTrue, GetTime());
				TransmitSpeed(0.0f,GetTime());
				if (!FinishedTransmitted)
				{
					if(!BlockTransmit) {
						TransmitBool(tTrue, GetTime());
					FinishedTransmitted = tTrue;
					}
				}
			}
			if (DistanceDrivenIgnoreTwo > m_filterProperties.ignoreDistance && !IgnoreTwoTransmitted)
				{
				TransmitIgnoreTwo(tTrue);
				DistanceDrivenIgnoreTwo = 0.0f;
				IgnoreTwoTransmitted = tTrue;
				}

			if (DistanceDrivenIgnoreThree > m_filterProperties.ignoreThreeDistance && !IgnoreThreeTransmitted)
				{
				TransmitIgnoreThree(tTrue);
				DistanceDrivenIgnoreThree = 0.0f;
				IgnoreThreeTransmitted = tTrue;
				}
			BlockTransmit = tFalse;

		}
	}
	RETURN_NOERROR;
}


tResult DriveDistance::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
	if (m_filterProperties.debugOutput) LOG_INFO(cString::Format("DriveDistanceSpeedOut: %f", speed));
	__synchronized_obj(CritSpeedOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalValue1->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue1, pMediaSample, pCoderOutput);

		if (!m_szIDOutputSpeedSet)
		{
			pCoderOutput->GetID("f32Value",m_szIDOutputSpeedControllerValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDOutputSpeedControllerTimeStamp);
			m_szIDOutputSpeedSet = tTrue;
		}
		pCoderOutput->Set(m_szIDOutputSpeedControllerValue, (tVoid*)&speed);
		pCoderOutput->Set(m_szIDOutputSpeedControllerTimeStamp, (tVoid*)&timestamp);


	pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputSpeedController.Transmit(pMediaSample));
	// cout << "DD speed : "<< speed << endl;
	RETURN_NOERROR;
}


tResult DriveDistance::ProcessBool(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritBoolIn);
	tBool Clear;
	tTimeStamp ts;
	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

		if(!m_szIDInputBoolSet)
		{
			pCoderInput->GetID("bValue",m_szIdBoolInValue);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDBoolTs);
			m_szIDInputBoolSet = tTrue;
		}

		pCoderInput->Get(m_szIdBoolInValue, (tVoid*)&Clear);
		pCoderInput->Get(m_szIDBoolTs, (tVoid*)&ts);
	}
	if(Clear)
	{
		DistanceToDrive.clear();
		OverallDistanceSet = tFalse;
	}
	RETURN_NOERROR;
}

tResult DriveDistance::ProcessIgnore(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritIgnoreIn);
	tBool Bool;
	tTimeStamp ts;
	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

		if(!m_szIDInputIgnoreBoolSet)
		{
			pCoderInput->GetID("bValue",m_szIdIgnoreBoolInValue);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDIgnoreBoolTs);
			m_szIDInputIgnoreBoolSet = tTrue;
		}

		pCoderInput->Get(m_szIdIgnoreBoolInValue, (tVoid*)&Bool);
		pCoderInput->Get(m_szIDIgnoreBoolTs, (tVoid*)&ts);
	}
	if(Bool)
	{
		OverallDistanceIgnoreSet = tFalse;
		IgnoreTransmitted = tFalse;
		DistanceDrivenIgnore = 0;
	}
	RETURN_NOERROR;
}

tResult DriveDistance::ProcessIgnoreTwo(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritIgnoreTwoIn);
	tBool Bool;
	tTimeStamp ts;
	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

		if(!m_szIDInputIgnoreTwoBoolSet)
		{
			pCoderInput->GetID("bValue",m_szIdIgnoreTwoBoolInValue);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDIgnoreTwoBoolTs);
			m_szIDInputIgnoreTwoBoolSet = tTrue;
		}

		pCoderInput->Get(m_szIdIgnoreTwoBoolInValue, (tVoid*)&Bool);
		pCoderInput->Get(m_szIDIgnoreTwoBoolTs, (tVoid*)&ts);
	}
	if(Bool)
	{
		OverallDistanceIgnoreTwoSet = tFalse;
		IgnoreTwoTransmitted = tFalse;
		DistanceDrivenIgnoreTwo = 0;
	}
	RETURN_NOERROR;
}

tResult DriveDistance::ProcessIgnoreThree(IMediaSample* pMediaSample)
{
	tBool Bool;
	tTimeStamp ts;
	__synchronized_obj(CritIgnoreThreeIn);
	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

		if(!m_szIDInputIgnoreThreeBoolSet)
		{
			pCoderInput->GetID("bValue",m_szIdIgnoreThreeBoolInValue);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDIgnoreThreeBoolTs);
			m_szIDInputIgnoreThreeBoolSet = tTrue;
		}

		pCoderInput->Get(m_szIdIgnoreThreeBoolInValue, (tVoid*)&Bool);
		pCoderInput->Get(m_szIDIgnoreThreeBoolTs, (tVoid*)&ts);
	}
		OverallDistanceIgnoreThreeSet = tFalse;
		IgnoreThreeTransmitted = tFalse;
		DistanceDrivenIgnoreThree = 0;
	RETURN_NOERROR;
}


tResult DriveDistance::ProcessSpeed(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritSpeedIn);
	tTimeStamp ts;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue5, pMediaSample, pCoderInput);

		if(!m_szIDInputSpeedSet)
		{
			pCoderInput->GetID("f32Value",m_szIdSpeedInValue);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDSpeedTs);
			m_szIDInputSpeedSet = tTrue;
		}

		pCoderInput->Get(m_szIdSpeedInValue, (tVoid*)&currentSpeed);
		pCoderInput->Get(m_szIDSpeedTs, (tVoid*)&ts);
	}
	RETURN_NOERROR;
}


tResult DriveDistance::ProcessDistanceOverall(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritDistanceOverall);
	tFloat32 lastDistance;
	tTimeStamp ts;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue2, pMediaSample, pCoderInput);

		if(!m_szIDDistanceOverallSet)
		{
			pCoderInput->GetID("f32Value",m_szIdDistanceOverall);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDDistanceOverallTs);
			m_szIDDistanceOverallSet = tTrue;
		}

		pCoderInput->Get(m_szIdDistanceOverall, (tVoid*)&lastDistance);
		pCoderInput->Get(m_szIDDistanceOverallTs, (tVoid*)&ts);
	}
	if (!OverallDistanceSet)
	{
		DistanceAtStart = lastDistance;
		OverallDistanceSet = tTrue;
		DistanceDriven = 0.0f;
	}
	else
	{
		DistanceDriven = lastDistance - DistanceAtStart;
	}
	if (!OverallDistanceIgnoreSet)
	{
		DistanceAtStartIgnore = lastDistance;
		OverallDistanceIgnoreSet = tTrue;
		DistanceDrivenIgnore = 0.0f;
	}
	else
	{
		DistanceDrivenIgnore = lastDistance - DistanceAtStartIgnore;
	}
	if (!OverallDistanceIgnoreTwoSet)
	{
		DistanceAtStartIgnoreTwo = lastDistance;
		OverallDistanceIgnoreTwoSet = tTrue;
		DistanceDrivenIgnoreTwo = 0.0f;
	}
	else
	{
		DistanceDrivenIgnoreTwo = lastDistance - DistanceAtStartIgnoreTwo;
	}
	if (!OverallDistanceIgnoreThreeSet)
	{
		DistanceAtStartIgnoreThree = lastDistance;
		OverallDistanceIgnoreThreeSet = tTrue;
		DistanceDrivenIgnoreThree = 0.0f;
	}
	else
	{
		DistanceDrivenIgnoreThree = lastDistance - DistanceAtStartIgnoreThree;
	}
	//cout << "DistanceDriven " << DistanceDriven << endl;
	RETURN_NOERROR;
}


tResult DriveDistance::ProcessDistanceToDrive(IMediaSample* pMediaSample)
{
	//if (!DriveInProcess)
	//{
__synchronized_obj(CritDistanceIn);
	BlockTransmit = tTrue;

	{

		__adtf_sample_read_lock_mediadescription(m_pDescriptionDriveDistance, pMediaSample, pCoderInput);

		if(!m_szIDInputDistanceSet)
		{
			pCoderInput->GetID("f32Value",m_szIdDistanceInValue);
			pCoderInput->GetID("bValue",m_szIDDistanceTs);
			m_szIDInputDistanceSet = tTrue;
		}

		pCoderInput->Get(m_szIdDistanceInValue, (tVoid*)&DistanceToDriveSigned);
		pCoderInput->Get(m_szIDDistanceTs, (tVoid*)&Brake);
	}
		if (DistanceToDriveSigned >= 0)
		{
			Reverse = tFalse;
			DistanceToDrive.push_back(DistanceToDriveSigned);
		}
		else
		{
			//cout << "Reverse reached" << endl;
			Reverse = tTrue;
			DistanceToDrive.push_back(-DistanceToDriveSigned);
		}
if(m_filterProperties.debugOutput) { 
 LOG_INFO(cString::Format("Distance %f received", DistanceToDriveSigned)); 
 }
	RETURN_NOERROR;
}


tResult DriveDistance::TransmitBool(tBool finished, tUInt32 timestamp)
{
	__synchronized_obj(CritBoolOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

		if (!m_szIDOutputBoolSet)
		{
			pCoderOutput->GetID("bValue",m_szIdBoolOutValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDBoolOutTs);
			m_szIDOutputBoolSet = tTrue;
		}
		pCoderOutput->Set(m_szIdBoolOutValue, (tVoid*)&finished);
		pCoderOutput->Set(m_szIDBoolOutTs, (tVoid*)&timestamp);


	pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputBool.Transmit(pMediaSample));

	RETURN_NOERROR;
}


tResult DriveDistance::TransmitBrakeLight(tBool finished, tUInt32 timestamp)
{
	__synchronized_obj(CritBrakeLightOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

		if (!m_szIDBrakeLightSet)
		{
			pCoderOutput->GetID("bValue",m_szIdBrakeLightOutValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDBrakeLightOutTs);
			m_szIDBrakeLightSet = tTrue;
		}
		pCoderOutput->Set(m_szIdBrakeLightOutValue, (tVoid*)&finished);
		pCoderOutput->Set(m_szIDBrakeLightOutTs, (tVoid*)&ts);

	pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputBrakeLight.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult DriveDistance::TransmitIgnore(tBool bvalue)
{
	__synchronized_obj(CritIgnoreOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

		if (!m_szIDOutputIgnoreBoolSet)
		{
			pCoderOutput->GetID("bValue",m_szIdIgnoreBoolOutValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDIgnoreBoolOutTs);
			m_szIDOutputIgnoreBoolSet = tTrue;
		}
		pCoderOutput->Set(m_szIdIgnoreBoolOutValue, (tVoid*)&bvalue);
		pCoderOutput->Set(m_szIDIgnoreBoolOutTs, (tVoid*)&ts);

	pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputIgnoreBool.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult DriveDistance::TransmitIgnoreTwo(tBool bvalue)
{
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

		if (!m_szIDOutputIgnoreTwoBoolSet)
		{
			pCoderOutput->GetID("bValue",m_szIdIgnoreTwoBoolOutValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDIgnoreTwoBoolOutTs);
			m_szIDOutputIgnoreTwoBoolSet = tTrue;
		}
		pCoderOutput->Set(m_szIdIgnoreTwoBoolOutValue, (tVoid*)&bvalue);
		pCoderOutput->Set(m_szIDIgnoreTwoBoolOutTs, (tVoid*)&ts);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());
	RETURN_IF_FAILED(m_oOutputIgnoreTwoBool.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult DriveDistance::TransmitIgnoreThree(tBool bvalue)
{
	__synchronized_obj(CritIgnoreThreeOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

		if (!m_szIDOutputIgnoreThreeBoolSet)
		{
			pCoderOutput->GetID("bValue",m_szIdIgnoreThreeBoolOutValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDIgnoreThreeBoolOutTs);
			m_szIDOutputIgnoreThreeBoolSet = tTrue;
		}
		pCoderOutput->Set(m_szIdIgnoreThreeBoolOutValue, (tVoid*)&bvalue);
		pCoderOutput->Set(m_szIDIgnoreThreeBoolOutTs, (tVoid*)&ts);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());
	RETURN_IF_FAILED(m_oOutputIgnoreThreeBool.Transmit(pMediaSample));

	RETURN_NOERROR;
}
tResult DriveDistance::PropertyChanged(const tChar* strName)
{
	RETURN_IF_FAILED(cFilter::PropertyChanged(strName)); 
 if (cString::IsEqual(strName, "DEBUG::DebutOutput")) 
 { 
 m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");  
 } 

	//associate the properties to the member
	if(cString::IsEqual(strName, "DefaultSpeed"))
		m_filterProperties.defaultSpeed = GetPropertyFloat("DefaultSpeed");
	else if(cString::IsEqual(strName, "BrakeDistance"))
		m_filterProperties.brakeDistance = GetPropertyFloat("BrakeDistance");
	else if(cString::IsEqual(strName, "WheelDiamater"))
		m_filterProperties.wheelDiameter = GetPropertyFloat("WheelDiamater");
	else if(cString::IsEqual(strName, "IgnoreDistance"))
		m_filterProperties.ignoreDistance = GetPropertyFloat("IgnoreDistance");
	else if(cString::IsEqual(strName, "IgnoreTwoDistance"))
		m_filterProperties.ignoreTwoDistance = GetPropertyFloat("IgnoreTwoDistance");
	else if(cString::IsEqual(strName, "IgnoreThreeDistance"))
		m_filterProperties.ignoreThreeDistance = GetPropertyFloat("IgnoreThreeDistance");
	RETURN_NOERROR;
}


tUInt32 DriveDistance::GetTime()
{
	return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}


tFloat32 DriveDistance::Betrag(tFloat32 input)
{
	tFloat32 Betrag = (input >= 0) ? input : -input;
	return Betrag;
}


