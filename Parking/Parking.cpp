#include <math.h>
#include "stdafx.h"
#include "Parking.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <string>


/// Create filter shell
ADTF_FILTER_PLUGIN("Parking", OID_ADTF_FILTER_DEF, Parking);


Parking::Parking(const tChar* __info):cFilter(__info)
{
	m_szIDsInputParkingFinishedSet = tFalse;
	m_szIDsInputParkingStartSet = tFalse;
	m_StartParking = tFalse;
}

Parking::~Parking()
{

}

tResult Parking::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

													// in StageFirst you can create and register your static pins.
																					if (eStage == StageFirst)
																					{
																						cObjectPtr<IMediaDescriptionManager> pDescManager;
																						RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
																						//media type für bools
																						tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
																						RETURN_IF_POINTER_NULL(strDescBool);
																						cObjectPtr<IMediaType> pTypeSignalBool = new cMediaType(0,0,0,"tBoolSignalValue",strDescBool, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

																						RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean));

																						//Pins
																						RETURN_IF_FAILED(m_oInputParkingStart.Create("StartParkingBoolIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
																						RETURN_IF_FAILED(RegisterPin(&m_oInputParkingStart));
																						RETURN_IF_FAILED(m_oOutputParkingFinished.Create("FinishedParkingBoolOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
																						RETURN_IF_FAILED(RegisterPin(&m_oOutputParkingFinished));
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

tResult Parking::Shutdown(tInitStage eStage, __exception)
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

tResult Parking::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	// if( GetMinimaMean(m_szIdUsMinima) < m_filterProperties.stopDist) {
	//cout << "Stopped\n" << endl;
	// }


	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		if (pSource == &m_oInputParkingStart)
		{
			ProcessParking(pMediaSample);
			ParseRoute();
		}
	}

	RETURN_NOERROR;
}

tResult Parking::ParseRoute()
{

	string line;
	ifstream myfile( "/home/aadc/Desktop/example.txt" );
	if (myfile)  // same as: if (myfile.good())
	{
		while (getline( myfile, line ))  // same as: while (getline( myfile, line ).good())
		{
			char * pch;
			char str[1024];
			strncpy(str, line.c_str(), sizeof(str));
			str[sizeof(str) - 1] = 0;

			pch = strtok (str," ");
			while (pch != NULL)
			{
				LOG_INFO(cString::Format("%s\n",pch));
				pch = strtok (NULL, " ");
			}
		}
		myfile.close();
	}
	RETURN_NOERROR;
}

tResult Parking::ProcessParking(IMediaSample* pMediaSample)
{
	__synchronized_obj(m_critSecStart);
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

		if(!m_szIDsInputParkingStartSet)
		{
			pCoderInput->GetID("bValue",m_szIdParkingStart);
			m_szIDsInputParkingStartSet = tTrue;
		}

		pCoderInput->Get(m_szIdParkingStart, (tVoid*)&m_StartParking);
	}
	RETURN_NOERROR;
}

tResult Parking::TransmitParking(tBool value)
{
	__synchronized_obj(m_critSecFinish);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

		if (!m_szIDsInputParkingFinishedSet)
		{
			pCoderOutput->GetID("bValue",m_szIdParkingFinished);
			m_szIDsInputParkingFinishedSet = tTrue;
		}
		pCoderOutput->Set(m_szIdParkingFinished, (tVoid*)&value);

		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputParkingFinished.Transmit(pMediaSample));


	RETURN_NOERROR;
}

tResult Parking::PropertyChanged(const tChar* strName)
{


	RETURN_NOERROR;
}





tUInt32 Parking::GetTime()
{
	return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 Parking::Betrag(tFloat32 input)
{
	tFloat32 Betrag = (input >= 0) ? input : -input;
	return Betrag;
}

