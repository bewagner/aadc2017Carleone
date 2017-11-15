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
#include <math.h>
#include "stdafx.h"
#include "functions.h"
#include "cRoadsignReaction.h"
#include <iostream>
//#include <algorithm>
//#include <vector>

/// Create filter shell
ADTF_FILTER_PLUGIN("RoadsignReaction", OID_ADTF_ROADSIGN_REACTION, cRoadsignReaction);

cRoadsignReaction::cRoadsignReaction(const tChar* __info) :
		cFilter(__info) {
	SetPropertyBool("DEBUG::DebugOutput", tFalse);
	SetPropertyStr("DEBUG::DebugOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebugOutput" NSSUBPROP_ISCHANGEABLE, tTrue);

	m_RoadsignExtBufferSet = tFalse;
	m_RoadsignBoolBufferSet = tFalse;
	m_CarPositionBufferSet = tFalse;
	m_ParkingBoolBufferSet = tFalse;

	m_szIdOutputStoppingSet = tFalse;
	m_szIdOutputParkingSet = tFalse;
	m_szIDFirstPositionReceivedSet = tFalse;
	m_oRoadsignOutputBufferSet = tFalse;
	m_szIdOutputCrossingSet = tFalse;
	m_szIdOutputSlowlySet = tFalse;
	m_szIdOutputRechtsVorLinksSet = tFalse;
	m_szIdOutputVorfahrtSet = tFalse;
	m_szIdOutputZebraSignSet = tFalse;

	currentCarPositionX = 0.0f;
	currentCarPositionY = 0.0f;
	currentCarDirection = 0.0f;
	FirstPositionReceived = tFalse;
	configLoaded = tFalse;
	RoadsignBool = tFalse;

	currentRoadsign.RoadsignID = -1000;
	currentRoadsign.RoadsignX = -1000;
	currentRoadsign.RoadsignY = -1000;
	currentRoadsign.RoadsignDirection = -1000;
	currentRoadsign.Roadsignseen = tFalse;

	DEG2RAD = (M_PI / 180.0);

	SetPropertyStr("Configuration", "roadSign.xml");
	SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
	SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");

	SetPropertyFloat("DISTANCE::RoadsignToRoadsignDist", 2.5f);
	SetPropertyStr("DISTANCE::RoadsignToRoadsignDist" NSSUBPROP_DESCRIPTION, "Distance between same Roadsigns");
	SetPropertyBool("DISTANCE::RoadsignToRoadsignDist" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("DISTANCE::neighborhoodDist", 2.0f);
	SetPropertyStr("DISTANCE::neighborhoodDist" NSSUBPROP_DESCRIPTION, "Neighborhood around the car");
	SetPropertyBool("DISTANCE::neighborhoodDist" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("DISTANCE::TransmitCrossingDistance", 1.0f);
	SetPropertyStr("DISTANCE::TransmitCrossingDistance" NSSUBPROP_DESCRIPTION, "TransmitCrossingDistance");
	SetPropertyBool("DISTANCE::TransmitCrossingDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("DISTANCE::TransmitSlowDistance", 2.0f);
	SetPropertyStr("DISTANCE::TransmitSlowDistance" NSSUBPROP_DESCRIPTION, "TransmitSlowDistance");
	SetPropertyBool("DISTANCE::TransmitSlowDistance" NSSUBPROP_ISCHANGEABLE, tTrue);
}

cRoadsignReaction::~cRoadsignReaction() {

}

tResult cRoadsignReaction::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		// create the description manager
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		// create the description for the road sign extended pin
		tChar const * strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");
		RETURN_IF_POINTER_NULL(strDescExt);
		cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		//media type für bools
		tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBool);
		cObjectPtr<IMediaType> pTypeSignalBool = new cMediaType(0,0,0,"tBoolSignalValue",strDescBool, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		//media type für bools
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0,0,0,"tSignalValue",strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		//media type für position
		tChar const * strDescPos = pDescManager->GetMediaDescription("tPosition");
		RETURN_IF_POINTER_NULL(strDescPos);
		cObjectPtr<IMediaType> pTypePos = new cMediaType(0, 0, 0, "tPosition", strDescPos, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		//media type für trafficsign
		tChar const * strDescTrafficSign = pDescManager->GetMediaDescription("tTrafficSign");
		RETURN_IF_POINTER_NULL(strDescTrafficSign);
		cObjectPtr<IMediaType> pTypeTrafficSign = new cMediaType(0, 0, 0, "tTrafficSign", strDescTrafficSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		RETURN_IF_FAILED(pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSignExt));
		RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalBool));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
		RETURN_IF_FAILED(pTypePos->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPos));
		RETURN_IF_FAILED(pTypeTrafficSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTrafficSign));

		//INPUT
		// create the roadsign extended InputPin
		RETURN_IF_FAILED(m_RoadsignExtInput.Create("RoadSignExt", pTypeExt, this));
		RETURN_IF_FAILED(RegisterPin(&m_RoadsignExtInput));

		// create the Position InputPin
		RETURN_IF_FAILED(m_CarPositionInput.Create("CarPosition", pTypePos, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_CarPositionInput));

		// create the RoadsignBool InputPin
		RETURN_IF_FAILED(m_RoadsignBoolInput.Create("RoadsignBool", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_RoadsignBoolInput));

		// create the ParkingBool InputPin
		RETURN_IF_FAILED(m_ParkingBoolInput.Create("ParkingBool", pTypeSignalBool, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_ParkingBoolInput));

		//OUTPUT
		// create the Stopping OutputPin
		RETURN_IF_FAILED(m_oOutputStopping.Create("Stopping", pTypeSignalBool, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputStopping));

		RETURN_IF_FAILED(m_oOutputVorfahrt.Create("Vorfahrt", pTypeSignalBool, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputVorfahrt));

		RETURN_IF_FAILED(m_oOutputRechtsVorLinks.Create("RechtsVorLinks", pTypeSignalBool, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputRechtsVorLinks));

		// create the Stopping OutputPin
		RETURN_IF_FAILED(m_oOutputCrossing.Create("Crossing", pTypeSignalBool, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputCrossing));

		RETURN_IF_FAILED(m_oOutputSlowly.Create("DriveSlowly", pTypeSignalBool, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputSlowly));

		// create the Stopping OutputPin
		RETURN_IF_FAILED(m_oOutputParking.Create("Parking", pTypeSignalBool, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputParking));

		RETURN_IF_FAILED(m_oOutputZebraSign.Create("ZabraSign", pTypeSignalBool, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputZebraSign));

		// create the BrakeLight OutputPin
		RETURN_IF_FAILED(m_oOutputFirstPositionReceived.Create("FirstPositionReceived",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputFirstPositionReceived));

		// create the Roadsign OutputPin
		RETURN_IF_FAILED(m_oRoadsignOutput.Create("TrafficSign", pTypeTrafficSign, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oRoadsignOutput));

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

tResult cRoadsignReaction::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initialized in the corresponding stage during Init.
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

tResult cRoadsignReaction::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample) {
	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		/*if (pSource == &m_RoadsignInput)
		 {
		 //ProcessRoadsign(pMediaSample);
		 }*/
		if (pSource == &m_RoadsignBoolInput) {
			ProcessRoadsignBool(pMediaSample);
			if (!RoadsignBool) {
				currentRoadsign.RoadsignID = -1000;
				currentRoadsign.RoadsignX = -1000;
				currentRoadsign.RoadsignY = -1000;
				currentRoadsign.RoadsignDirection = -1000;
				currentRoadsign.Roadsignseen = tFalse;
			}
		}
		if (pSource == &m_RoadsignExtInput) {
			if (FirstPositionReceived) {
				ProcessRoadsignExt(pMediaSample);
			}
		}
		if (pSource == &m_ParkingBoolInput) {
				ProcessParkingBool(pMediaSample);
		}
		if (pSource == &m_CarPositionInput) {
			ProcessCarPosition(pMediaSample);

			if (!configLoaded) {
				LoadConfiguration();
				configLoaded = tTrue;
			}

			if (!FirstPositionReceived) {
				RETURN_NOERROR;
			}
			// Hier checken wir die aktuelle Nachbarschaft. Wir fügen Schilder hinzu, die vorher noch nicht da waren und löschen Schilder,
			//  die nicht mehr da sind.
			CheckNeighborhood();
			ResetRoadsignseenBool();
		}
	}

	RETURN_NOERROR;
}

tResult cRoadsignReaction::ProcessRoadsignExt(IMediaSample* pMediaSample) {

	__synchronized_obj(CritRoadsignExtIn);

	//cv::Vec3d currentRvec,currentTvec;
	tInt16 currentRoadsignID;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionRoadSignExt, pMediaSample, pCoderInput);

		if (!m_RoadsignExtBufferSet) {
			pCoderInput->GetID("i16Identifier", m_RoadsignExtBufferID);
			pCoderInput->GetID("af32RVec[0]", m_RoadsignExtRvec0BufferID);
			pCoderInput->GetID("af32RVec[1]", m_RoadsignExtRvec1BufferID);
			pCoderInput->GetID("af32RVec[2]", m_RoadsignExtRvec2BufferID);
			pCoderInput->GetID("af32TVec[0]", m_RoadsignExtTvec0BufferID);
			pCoderInput->GetID("af32TVec[1]", m_RoadsignExtTvec1BufferID);
			pCoderInput->GetID("af32TVec[2]", m_RoadsignExtTvec2BufferID);
			m_RoadsignExtBufferSet = tTrue;
		}
		pCoderInput->Get(m_RoadsignExtBufferID, (tVoid*) &currentRoadsignID);
		pCoderInput->Get(m_RoadsignExtRvec0BufferID, (tVoid*) &currentRvec[0]);
		pCoderInput->Get(m_RoadsignExtRvec1BufferID, (tVoid*) &currentRvec[1]);
		pCoderInput->Get(m_RoadsignExtRvec2BufferID, (tVoid*) &currentRvec[2]);
		pCoderInput->Get(m_RoadsignExtTvec0BufferID, (tVoid*) &currentTvec[0]);
		pCoderInput->Get(m_RoadsignExtTvec1BufferID, (tVoid*) &currentTvec[1]);
		pCoderInput->Get(m_RoadsignExtTvec2BufferID, (tVoid*) &currentTvec[2]);
	}

	double r = sqrt(currentTvec[2] * currentTvec[2] + currentTvec[1] * currentTvec[1]);
	double theta = normalizeAngle(currentRvec[2] + currentCarDirection, 0.0f);

	currentRoadsign.RoadsignID = currentRoadsignID;
	currentRoadsign.RoadsignX = currentCarPositionX + cos(theta) * r;
	currentRoadsign.RoadsignY = currentCarPositionY + sin(theta) * r;
	currentRoadsign.RoadsignDirection = normalizeAngle(currentCarDirection + M_PI, 0.0f);
	currentRoadsign.Roadsignseen = tFalse;

	//cout << "Car X:" << currentCarPositionX << endl;
	//cout << "Car Y:" << currentCarPositionY << endl;
	//cout << "RS X:" << currentRoadsign.RoadsignX << endl;
	//cout << "RS Y:" << currentRoadsign.RoadsignY << endl;
	//cout << currentCarDirection << endl;
	if (currentTvec[2] < m_filterProperties.TransmitCrossingDistance) {
		switch (currentRoadsignID) {
		case 0:
			TransmitCrossing(tTrue);
			break;
		case 1:
			TransmitCrossing(tTrue);
			break;
		case 2:
			if(ParkingBool)	TransmitParking(tTrue);
			break;
		case 3:
			TransmitCrossing(tTrue);
			break;
		case 5:
			TransmitCrossing(tTrue);
			break;
		case 6:
			TransmitZebraSign(tTrue);
			break;
		default:
			break;
		}
	}

	//  TODO Was ist Schild 4?

	if (currentTvec[2] < m_filterProperties.TransmitSlowDistance) {
		switch (currentRoadsignID) {
		case 0:
			TransmitSlowly(tTrue);
			TransmitRechtsVorLinks(tTrue);
			break;
		case 1:
			TransmitSlowly(tTrue);
			TransmitRechtsVorLinks(tFalse);
			TransmitVorfahrt(tFalse);
			break;
		case 3:
			TransmitSlowly(tTrue);
			TransmitRechtsVorLinks(tFalse);
			TransmitVorfahrt(tTrue);
			break;
		case 5:
			TransmitSlowly(tTrue);
			TransmitRechtsVorLinks(tFalse);
			TransmitVorfahrt(tFalse);
			break;
		case 6:
			//TransmitSlowly(tTrue);
			TransmitZebraSign(tTrue);
			break;
		default:
			break;
		}
	}
	RETURN_NOERROR;

}

tResult cRoadsignReaction::ProcessCarPosition(IMediaSample* pMediaSample) {
	__synchronized_obj(CritCarPositionIn);
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionPos, pMediaSample, pCoderInput);

		if (!m_CarPositionBufferSet) {
			pCoderInput->GetID("f32x", m_CarPositionXBufferID);
			pCoderInput->GetID("f32y", m_CarPositionYBufferID);
			pCoderInput->GetID("f32heading", m_CarPositionDirectionBufferID);
			m_CarPositionBufferSet = tTrue;
		}
		pCoderInput->Get(m_CarPositionXBufferID, (tVoid*) &currentCarPositionX);
		pCoderInput->Get(m_CarPositionYBufferID, (tVoid*) &currentCarPositionY);
		pCoderInput->Get(m_CarPositionDirectionBufferID, (tVoid*) &currentCarDirection);
	}
	if (!FirstPositionReceived) {
		FirstPositionReceived = tTrue;
		TransmitFirstPositionReceived(tTrue);
	}
	RETURN_NOERROR;
}

tResult cRoadsignReaction::ProcessRoadsignBool(IMediaSample* pMediaSample) {
	__synchronized_obj(CritRoadsignBoolIn);
	tTimeStamp ts;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

		if (!m_RoadsignBoolBufferSet) {
			pCoderInput->GetID("bValue", m_RoadsignBoolBufferID);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_RoadsignBoolBufferIDTs);
			m_RoadsignBoolBufferSet = tTrue;
		}
		pCoderInput->Get(m_RoadsignBoolBufferID, (tVoid*) &RoadsignBool);
		pCoderInput->Get(m_RoadsignBoolBufferIDTs, (tVoid*) &ts);
	}
	RETURN_NOERROR;
}

tResult cRoadsignReaction::ProcessParkingBool(IMediaSample* pMediaSample) {
	__synchronized_obj(CritParkingBoolIn);
	tTimeStamp ts;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderInput);

		if (!m_ParkingBoolBufferSet) {
			pCoderInput->GetID("bValue", m_ParkingBoolBufferID);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_ParkingBoolBufferIDTs);
			m_ParkingBoolBufferSet = tTrue;
		}
		pCoderInput->Get(m_ParkingBoolBufferID, (tVoid*) &ParkingBool);
		pCoderInput->Get(m_ParkingBoolBufferIDTs, (tVoid*) &ts);
	}
	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitStopping(tBool stopBool) {
	__synchronized_obj(CritStoppingOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIdOutputStoppingSet) {
			pCoderOutput->GetID("bValue", m_szIdOutputStopping);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputStoppingTs);
			m_szIdOutputStoppingSet = tTrue;
		}
		pCoderOutput->Set(m_szIdOutputStopping, (tVoid*) &stopBool);
		pCoderOutput->Set(m_szIdOutputStoppingTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputStopping.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitVorfahrt(tBool bValue) {
	__synchronized_obj(CritVorfahrtOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIdOutputVorfahrtSet) {
			pCoderOutput->GetID("bValue", m_szIdOutputVorfahrt);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputVorfahrtTs);
			m_szIdOutputVorfahrtSet = tTrue;
		}
		pCoderOutput->Set(m_szIdOutputVorfahrt, (tVoid*) &bValue);
		pCoderOutput->Set(m_szIdOutputVorfahrtTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputVorfahrt.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitRechtsVorLinks(tBool bValue) {
	__synchronized_obj(CritRechtsvorLinksOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIdOutputRechtsVorLinksSet) {
			pCoderOutput->GetID("bValue", m_szIdOutputRechtsVorLinks);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputRechtsVorLinksTs);
			m_szIdOutputRechtsVorLinksSet = tTrue;
		}
		pCoderOutput->Set(m_szIdOutputRechtsVorLinks, (tVoid*) &bValue);
		pCoderOutput->Set(m_szIdOutputRechtsVorLinksTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputRechtsVorLinks.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitCrossing(tBool stopBool) {
	__synchronized_obj(CritCrossingOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIdOutputCrossingSet) {
			pCoderOutput->GetID("bValue", m_szIdOutputCrossing);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputCrossingTs);
			m_szIdOutputCrossingSet = tTrue;
		}
		pCoderOutput->Set(m_szIdOutputCrossing, (tVoid*) &stopBool);
		pCoderOutput->Set(m_szIdOutputCrossingTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputCrossing.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitSlowly(tBool value) {
	__synchronized_obj(CritSlowlyOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIdOutputSlowlySet) {
			pCoderOutput->GetID("bValue", m_szIdOutputSlowly);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSlowlyTs);
			m_szIdOutputSlowlySet = tTrue;
		}
		pCoderOutput->Set(m_szIdOutputSlowly, (tVoid*) &value);
		pCoderOutput->Set(m_szIdOutputSlowlyTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputSlowly.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitParking(tBool parkingBool) {
	__synchronized_obj(CritParkingOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIdOutputParkingSet) {
			pCoderOutput->GetID("bValue", m_szIdOutputParking);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputParkingTs);

			m_szIdOutputParkingSet = tTrue;
		}
		pCoderOutput->Set(m_szIdOutputParking, (tVoid*) &parkingBool);
		pCoderOutput->Set(m_szIdOutputParkingTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputParking.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitZebraSign(tBool ZebraSignBool) {
	__synchronized_obj(CritZebraOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIdOutputZebraSignSet) {
			pCoderOutput->GetID("bValue", m_szIdOutputZebraSign);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputZebraSignTs);
			m_szIdOutputZebraSignSet = tTrue;
		}
		pCoderOutput->Set(m_szIdOutputZebraSign, (tVoid*) &ZebraSignBool);
		pCoderOutput->Set(m_szIdOutputZebraSignTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputZebraSign.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitFirstPositionReceived(tBool brakeLight) {
	__synchronized_obj(CritFirstPositionReceivedOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
	tTimeStamp ts = GetTime();
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalBool->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalBool, pMediaSample, pCoderOutput);

		if (!m_szIDFirstPositionReceivedSet) {
			pCoderOutput->GetID("bValue", m_szIdFirstPositionReceived);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdFirstPositionReceivedTs);
			m_szIDFirstPositionReceivedSet = tTrue;
		}
		pCoderOutput->Set(m_szIdFirstPositionReceived, (tVoid*) &brakeLight);
		pCoderOutput->Set(m_szIdFirstPositionReceivedTs, (tVoid*) &ts);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputFirstPositionReceived.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult cRoadsignReaction::TransmitTrafficSign(RoadsignStruct currentRoadsign) {
	__synchronized_obj(CritTrafficSignOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionTrafficSign->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	if (configLoaded) {
		tInt16 ID = currentRoadsign.RoadsignID;
		tFloat32 X = currentRoadsign.RoadsignX;
		tFloat32 Y = currentRoadsign.RoadsignY;
		tFloat32 Direction = currentRoadsign.RoadsignDirection;

		{
			__adtf_sample_write_lock_mediadescription(m_pDescriptionTrafficSign, pMediaSample, pCoderOutput);

			if (!m_oRoadsignOutputBufferSet) {
				pCoderOutput->GetID("i16Identifier", m_oRoadsignIDOutputBufferID);
				pCoderOutput->GetID("f32x", m_oRoadsignXOutputBufferID);
				pCoderOutput->GetID("f32y", m_oRoadsignYOutputBufferID);
				pCoderOutput->GetID("f32angle", m_oRoadsignDirectionOutputBufferID);
				m_oRoadsignOutputBufferSet = tTrue;
			}
			pCoderOutput->Set(m_oRoadsignIDOutputBufferID, (tVoid*) &ID);
			pCoderOutput->Set(m_oRoadsignXOutputBufferID, (tVoid*) &X);
			pCoderOutput->Set(m_oRoadsignYOutputBufferID, (tVoid*) &Y);
			pCoderOutput->Set(m_oRoadsignDirectionOutputBufferID, (tVoid*) &Direction);

			pMediaSample->SetTime(_clock->GetStreamTime());
		}
		RETURN_IF_FAILED(m_oRoadsignOutput.Transmit(pMediaSample));
		PRINT4("ROADSIGN: Wir transmiten Schild mit ID %i. X: %f, Y: %f, Direction: %f", ID, X, Y, Direction);
		PRINT1("ROADSIGN: seenRoadsigns hat Größe: %i", seenRoadsigns.size());
		PRINT3("ROADSIGN: Das Auto steht an X: %f, Y: %f, Direction: %f", currentCarPositionX, currentCarPositionY, currentCarDirection);

	}
	RETURN_NOERROR;

}

tResult cRoadsignReaction::CheckNeighborhood() {
	//Diese Funktion compiliert nicht wegen dem iterator :(
	/*
	 int i = 0;
	 vector<int> indexMissingRoadsign;
	 tBool currentRoadSignFound = tFalse;
	 tBool unwantedID = currentRoadsign.RoadsignID == -1000 || currentRoadsign.RoadsignID == 10;

	 if (!unwantedID) {
	 // Hier gehen wir durch den Vektor aller bekannter Schilder und schauen, ob wir sie sehen, wenn wir sie sehen müssten.
	 for (vector<RoadsignStruct>::iterator it = seenRoadsigns.begin(); it != seenRoadsigns.end(); ++it) {

	 if (!(currentRoadSignFound) && RoadsignBool) {
	 tFloat32 xDistance = Betrag(it->RoadsignX - currentRoadsign.RoadsignX);
	 tFloat32 yDistance = Betrag(it->RoadsignY - currentRoadsign.RoadsignY);
	 tFloat32 distance = sqrt(xDistance * xDistance + yDistance * yDistance);
	 tBool IDEqual = it->RoadsignID == currentRoadsign.RoadsignID;
	 //tFloat32 sameDirection = Betrag(it->RoadsignDirection - currentRoadsign.RoadsignDirection);

	 if (distance < m_filterProperties.RoadsignToRoadsignDist && IDEqual) {
	 it->Roadsignseen = tTrue;
	 currentRoadSignFound = tTrue;

	 //PRINT1("ROADSIGN: Wir sehen ein Schild aus der Liste mit ID: %i", currentRoadsign.RoadsignID);
	 }
	 }

	 tFloat32 schildZuAutoX = (it->RoadsignX - currentCarPositionX);
	 tFloat32 schildZuAutoY = (it->RoadsignY - currentCarPositionY);

	 tFloat32 autoAusrichtungX = cos(currentCarDirection);
	 tFloat32 autoAusrichtungY = sin(currentCarDirection);

	 tBool isRoadsignInFrontOfCar = schildZuAutoX * autoAusrichtungX + schildZuAutoY * autoAusrichtungY > 0;

	 tFloat32 r = sqrt(schildZuAutoX * schildZuAutoX + schildZuAutoY * schildZuAutoY);

	 tBool schildZeigtAufAuto = (Betrag(mod((it->RoadsignDirection), 2 * M_PI) - currentCarDirection) < (M_PI / 5));

	 if (it->RoadsignID == 2) {

	 PRINT1("ROADSIGN: isRoadsignInFrontOfCar: %i", schildZuAutoX * autoAusrichtungX + schildZuAutoY * autoAusrichtungY);
	 PRINT4("ROADSIGN: Entfernung zum Auto: %f, isRoadsignInFrontOfCar: %i, schildZeigtAufAuto: %i, Roadsignseen: %i", r, isRoadsignInFrontOfCar, schildZeigtAufAuto, it->Roadsignseen)
	 PRINT3("ROADSIGN: Das Auto steht an X: %f, Y: %f, Direction: %f", currentCarPositionX, currentCarPositionY, currentCarDirection);
	 PRINT3("ROADSIGN: Das Schild steht an X: %f, Y: %f, Direction: %f", it->RoadsignX, it->RoadsignY, it->RoadsignDirection);
	 }
	 // Wenn das aktuelle Schild aus der Liste in unserer Nähe ist, sich vor dem Auto befindet und wir genau ein Schild sehen, dann schauen wir, ob es sich um das richtige Schild handelt.
	 if (r > 0.3 && r < m_filterProperties.neighborhoodDist && isRoadsignInFrontOfCar && schildZeigtAufAuto && !(it->Roadsignseen)) {
	 PRINT1("ROADSIGN: RoadsignBool: %i", RoadsignBool);
	 if (RoadsignBool) {

	 tFloat32 xDistance = Betrag(it->RoadsignX - currentRoadsign.RoadsignX);
	 tFloat32 yDistance = Betrag(it->RoadsignY - currentRoadsign.RoadsignY);
	 tFloat32 distance = sqrt(xDistance * xDistance + yDistance * yDistance);
	 tBool IDEqual = it->RoadsignID == currentRoadsign.RoadsignID;

	 if (!(it->Roadsignseen)) {
	 // Hier merken wir uns den Index des Schildes, das wir löschen müssen und löschen das Schild nach dem
	 // for-loop.

	 PRINT1("ROADSIGN: Distance zwischen gelöschtem Schild und gefundenem Schild %f", distance);
	 PRINT4("ROADSIGN: Wir sehen Schild mit ID %i, X: %f, Y: %f, Ausrichtung: %f", currentRoadsign.RoadsignID, currentRoadsign.RoadsignX, currentRoadsign.RoadsignY, currentRoadsign.RoadsignDirection);
	 PRINT3("ROADSIGN: Wir löschen Schild mit ID %i, ListenIndex: %i, Ausrichtung: %f", it->RoadsignID, i, it->RoadsignDirection);
	 indexMissingRoadsign.push_back(i);
	 it->RoadsignID = -1;
	 TransmitTrafficSign(*it);

	 }

	 } else {
	 // Hier sehen wir kein Schild; es sollte aber eins da sein. Daher löschen wir das Schild.

	 PRINT1("ROADSIGN: Distance zwischen gelöschtem Schild und Auto %f", r);
	 PRINT1("ROADSIGN: Wir sehen kein Schild hier müsste aber eins mit ID %i sein.", it->RoadsignID);
	 indexMissingRoadsign.push_back(i);
	 it->RoadsignID = -1;
	 TransmitTrafficSign(*it);
	 }
	 }
	 i++;
	 }

	 // Hier löschen wir alle Schilder, die wir uns vorher als löschbar gemerkt haben.
	 for (vector<int>::reverse_iterator it = indexMissingRoadsign.rbegin(); it != indexMissingRoadsign.rend(); ++it) {
	 // LOG_INFO(cString::Format("Schild löschen: %i", *it ));

	 seenRoadsigns.erase(seenRoadsigns.begin() + *it);

	 // LOG_INFO(cString::Format("SeenRoadsign Size: %d", seenRoadsigns.size()));

	 }
	 indexMissingRoadsign.clear();
	 tFloat32 currentDistance = sqrt((currentRoadsign.RoadsignX - currentCarPositionX) * (currentRoadsign.RoadsignX - currentCarPositionX) + (currentRoadsign.RoadsignY - currentCarPositionY) * (currentRoadsign.RoadsignY - currentCarPositionY));
	 // Wir sehen ein Schild, das aber noch nicht in unserer Liste war, daher fügen wir es hinzu.
	 if (!(currentRoadSignFound) && RoadsignBool && !unwantedID && currentDistance < 1) {
	 seenRoadsigns.push_back(currentRoadsign);
	 TransmitTrafficSign(currentRoadsign);
	 }

	 //  // Ausgeben aller Schilder zu Debuggingzwecken
	 // int j = 0;
	 // for (vector<RoadsignStruct>::iterator it = seenRoadsigns.begin() ; it != seenRoadsigns.end(); ++it) {
	 // 	LOG_INFO(cString::Format("%i Road sign: %d",j++, it->RoadsignID));
	 // }
	 }
	 */
	RETURN_NOERROR;
}

tResult cRoadsignReaction::ResetRoadsignseenBool() {
	for (vector<RoadsignStruct>::iterator it = seenRoadsigns.begin(); it != seenRoadsigns.end(); ++it) {

		tFloat32 schildZuAutoX = (it->RoadsignX - currentCarPositionX);
		tFloat32 schildZuAutoY = (it->RoadsignY - currentCarPositionY);
		tFloat32 r = sqrt(schildZuAutoX * schildZuAutoX + schildZuAutoY * schildZuAutoY);

		if (r > m_filterProperties.neighborhoodDist + 0.5) {
			it->Roadsignseen = tFalse;
		}
	}
	RETURN_NOERROR;
}

tResult cRoadsignReaction::LoadConfiguration() {
	cFilename fileConfig = GetPropertyStr("Configuration");

	// create absolute path for marker configuration file
	ADTF_GET_CONFIG_FILENAME(fileConfig);
	fileConfig = fileConfig.CreateAbsolutePath(".");

	if (fileConfig.IsEmpty()) {
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	tInt i = 0;
	if (cFileSystem::Exists(fileConfig)) {
		cDOM oDOM;
		oDOM.Load(fileConfig);
		cDOMElementRefList oElems;

		if (IS_OK(oDOM.FindNodes("configuration/roadSign", oElems))) {
			// cout << "IS_OK" << endl;
			for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem) {
				RoadsignStruct item;
				item.RoadsignID = tUInt16((*itElem)->GetAttribute("id", "0").AsInt32());
				item.RoadsignX = tFloat32((*itElem)->GetAttribute("x", "0").AsFloat64());
				item.RoadsignY = tFloat32((*itElem)->GetAttribute("y", "0").AsFloat64());
				item.RoadsignRadius = tFloat32((*itElem)->GetAttribute("radius", "0").AsFloat64());
				item.RoadsignDirection = tFloat32((*itElem)->GetAttribute("direction", "0").AsFloat64());

				//item.u16Cnt = 0;
				//item.u32ticks = GetTime();

				item.RoadsignDirection *= DEG2RAD; // convert to radians

				PRINT5("ROADSIGN: LoadConfiguration::Id %d XY %f %f Radius %f Direction %f", item.RoadsignID, item.RoadsignX, item.RoadsignY, item.RoadsignRadius, item.RoadsignDirection);

				//CheckRoadsign(item);
				if (item.RoadsignID != 10)
					seenRoadsigns.push_back(item);
				//cout << "Push Back Roadsign: ID " << item.RoadsignID << ", X"<< item.RoadsignX << ", Y"<< item.RoadsignY << ", Direction "<< item.RoadsignDirection << endl;
				//cout << "SeenRoadsign Size: " << seenRoadsigns.size() << endl;
				i++;

			}
		}
	} else {
		LOG_ERROR("Configuration file does not exist");
		RETURN_ERROR(ERR_INVALID_FILE);
	}
	PRINT1("ROADSIGN: SeenRoadsign Size: %d", seenRoadsigns.size());
	PRINT3("ROADSIGN: Das Auto steht an X: %f, Y: %f, Direction: %f", currentCarPositionX, currentCarPositionY, currentCarDirection);

	RETURN_NOERROR;
}

tTimeStamp cRoadsignReaction::GetTime() {
	return (_clock != NULL) ? _clock->GetTime() : cSystem::GetTime();
}
tFloat32 cRoadsignReaction::Betrag(tFloat32 input) {
	return (input >= 0) ? input : -input;

}
tResult cRoadsignReaction::PropertyChanged(const tChar* strName) {
	RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
	if (cString::IsEqual(strName, "DEBUG::DebugOutput")) {
		m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebugOutput");
	}
	//associate the properties to the member
	if (cString::IsEqual(strName, "DISTANCE::RoadsignToRoadsignDist"))
		m_filterProperties.RoadsignToRoadsignDist = GetPropertyFloat("DISTANCE::RoadsignToRoadsignDist");
	else if (cString::IsEqual(strName, "DISTANCE::neighborhoodDist"))
		m_filterProperties.neighborhoodDist = GetPropertyFloat("DISTANCE::neighborhoodDist");
	else if (cString::IsEqual(strName, "DISTANCE::TransmitCrossingDistance"))
		m_filterProperties.TransmitCrossingDistance = GetPropertyFloat("DISTANCE::TransmitCrossingDistance");
	else if (cString::IsEqual(strName, "DISTANCE::TransmitSlowDistance"))
		m_filterProperties.TransmitSlowDistance = GetPropertyFloat("DISTANCE::TransmitSlowDistance");
	RETURN_NOERROR;
}

tFloat32 cRoadsignReaction::normalizeAngle(tFloat32 alpha, tFloat32 center) {
	return mod(alpha - center + static_cast<tFloat32>(M_PI), 2.0 * static_cast<tFloat32>(M_PI)) + center - static_cast<tFloat32>(M_PI);
}

tFloat32 cRoadsignReaction::mod(tFloat32 x, tFloat32 y) {
	tFloat32 r;
	tFloat32 b_x;
	if (y == floor(y)) {
		return x - floor(x / y) * y;
	} else {
		r = x / y;
		if (r < 0.0f) {
			b_x = ceil(r - 0.5f);
		} else {
			b_x = floor(r + 0.5f);
		}
		if (fabs(r - b_x) <= 2.2204460492503131E-16f * fabs(r)) {
			return 0.0f;
		} else {
			return (r - floor(r)) * y;
		}
	}
}
