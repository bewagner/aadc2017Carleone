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
   * $Author:: spiesra $  $Date:: 2017-04-28 08:10:05#$ $Rev:: 62659   $
   **********************************************************************/
#include "cClassificationPostprocessing.h"
#include "functions.h"
#include <stdlib.h> 
#include <cmath> 
// define the ADTF property names to avoid errors 
ADTF_FILTER_PLUGIN(ADTF_BFFT_FILTER_DESC,
				   OID_ADTF_BFFT_FILTER_DEF,
				   cClassificationPostprocessing)

cClassificationPostprocessing::cClassificationPostprocessing(const tChar* __info) : cFilter(__info)
{
	SetPropertyBool("DEBUG::DebutOutput", tFalse); 
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?"); 
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	
	SetPropertyFloat("FILTERING::MinProbability", 75.0);
	SetPropertyStr("FILTERING::MinProbability" NSSUBPROP_DESCRIPTION, "The Minimum probability for a classified object to be taken into consideration.");
	SetPropertyBool("FILTERING::MinProbability" NSSUBPROP_ISCHANGEABLE, tTrue);
	
	SetPropertyFloat("CLASSIFICATION::LeftSidePercentage", 0.342);
	SetPropertyStr("CLASSIFICATION::LeftSidePercentage" NSSUBPROP_DESCRIPTION, "The percentage of the picture that is classified as left.");
	SetPropertyBool("CLASSIFICATION::LeftSidePercentage" NSSUBPROP_ISCHANGEABLE, tTrue);
	
	SetPropertyFloat("CLASSIFICATION::RightSidePercentage", 0.555);
	SetPropertyStr("CLASSIFICATION::RightSidePercentage" NSSUBPROP_DESCRIPTION, "The percentage of the picture that is classified as right.");
	SetPropertyBool("CLASSIFICATION::RightSidePercentage" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("CLASSIFICATION::PersonOnStreetLeftSidePercentage", 0.38);
	SetPropertyStr("CLASSIFICATION::PersonOnStreetLeftSidePercentage" NSSUBPROP_DESCRIPTION, "The percentage of the picture that is classified as left.");
	SetPropertyBool("CLASSIFICATION::PersonOnStreetLeftSidePercentage" NSSUBPROP_ISCHANGEABLE, tTrue);
	
	SetPropertyFloat("CLASSIFICATION::PersonOnStreetRightSidePercentage", 0.55);
	SetPropertyStr("CLASSIFICATION::PersonOnStreetRightSidePercentage" NSSUBPROP_DESCRIPTION, "The percentage of the picture that is classified as right.");
	SetPropertyBool("CLASSIFICATION::PersonOnStreetRightSidePercentage" NSSUBPROP_ISCHANGEABLE, tTrue);
    
    SetPropertyFloat("CLASSIFICATION::PersonOnStreetEmergencyStopDistance", 0.75);
	SetPropertyStr("CLASSIFICATION::PersonOnStreetEmergencyStopDistance" NSSUBPROP_DESCRIPTION, "Minimum distance when looking for persons on the street.");
	SetPropertyBool("CLASSIFICATION::PersonOnStreetEmergencyStopDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("CLASSIFICATION::QueueLength", 3);
	SetPropertyStr("CLASSIFICATION::QueueLength" NSSUBPROP_DESCRIPTION, "Length of crossing classification queue.");
	SetPropertyBool("CLASSIFICATION::QueueLength" NSSUBPROP_ISCHANGEABLE, tTrue);
	
	SetPropertyBool("CLASSIFICATION::KidClassification", tTrue);
	SetPropertyStr("CLASSIFICATION::KidClassification" NSSUBPROP_DESCRIPTION, "Do we want to classify kids?");
	SetPropertyBool("CLASSIFICATION::KidClassification" NSSUBPROP_ISCHANGEABLE, tTrue);
	
	SetPropertyInt("CLASSIFICATION::PersonOrientationThreshold", 1);
	SetPropertyStr("CLASSIFICATION::PersonOrientationThreshold" NSSUBPROP_DESCRIPTION, "How many persons we have to have seen before sending true.");
	SetPropertyBool("CLASSIFICATION::PersonOrientationThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);


	m_szIDZebraStreifenSet = tFalse;
	m_szIDCrossingSet = tFalse;
	m_szIDKidSeenSet = tFalse;
	m_szIDServerIsRunningSet = tFalse;
	m_szIDPersonOnStreetSeenSet = tFalse;
	// file.open("/home/aadc/Desktop/test.txt");
	//picture ="/home/aadc/Desktop/picture.txt";

	changed = false;

	left_sum = 0;
	right_sum = 0;
	center_sum = 0;
}

tResult cClassificationPostprocessing::PropertyChanged(const tChar* strName)
{

	RETURN_IF_FAILED(cFilter::PropertyChanged(strName)); 
	if (cString::IsEqual(strName, "DEBUG::DebutOutput")) 
	{ 
		m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");  
	} 

	//associate the properties to the member
	if (cString::IsEqual(strName, "FILTERING::MinProbability"))
		m_filterProperties.minProbability = GetPropertyFloat("FILTERING::MinProbability");
	else if (cString::IsEqual(strName, "CLASSIFICATION::KidClassification"))
		m_filterProperties.kidClassification = GetPropertyFloat("CLASSIFICATION::KidClassification");
	else if (cString::IsEqual(strName, "CLASSIFICATION::PersonOrientationThreshold"))
		m_filterProperties.personOrientationThreshold = GetPropertyFloat("CLASSIFICATION::PersonOrientationThreshold");
	else if (cString::IsEqual(strName, "CLASSIFICATION::LeftSidePercentage"))
		m_filterProperties.LeftSidePercentage = GetPropertyFloat("CLASSIFICATION::LeftSidePercentage");
	else if (cString::IsEqual(strName, "CLASSIFICATION::RightSidePercentage"))
		m_filterProperties.RightSidePercentage = GetPropertyFloat("CLASSIFICATION::RightSidePercentage");
	else if (cString::IsEqual(strName, "CLASSIFICATION::PersonOnStreetLeftSidePercentage"))
		m_filterProperties.PersonOnStreetLeftSidePercentage = GetPropertyFloat("CLASSIFICATION::PersonOnStreetLeftSidePercentage");
	else if (cString::IsEqual(strName, "CLASSIFICATION::PersonOnStreetRightSidePercentage"))
		m_filterProperties.PersonOnStreetRightSidePercentage = GetPropertyFloat("CLASSIFICATION::PersonOnStreetRightSidePercentage");
    else if (cString::IsEqual(strName, "CLASSIFICATION::PersonOnStreetEmergencyStopDistance")) {
        tFloat32 result = GetPropertyFloat("CLASSIFICATION::PersonOnStreetEmergencyStopDistance");
        m_filterProperties.PersonOnStreetEmergencyStopDistance =  result >= 0 ? result : 0;
    }
    else if (cString::IsEqual(strName, "CLASSIFICATION::QueueLength")) {
		
        // Anpassen an die neue Queue-Länge.
        int oldSize = left.size();
        m_filterProperties.CrossingClassificationQueueSize = GetPropertyFloat("CLASSIFICATION::QueueLength");
        int newSize = m_filterProperties.CrossingClassificationQueueSize;
        int queueSizeDifference =  abs(oldSize - newSize);
		
        for(int i = 0; i < queueSizeDifference; ++i) {
            if(newSize > oldSize) {
                left.push_back(0);
                center.push_back(0);
                right.push_back(0);
            } else {
                left.pop_front();
                center.pop_front();
                right.pop_front();
            }
        }
		
    }
	

	
	RETURN_NOERROR;
}

cClassificationPostprocessing::~cClassificationPostprocessing()
{
}

tResult cClassificationPostprocessing::Start(__exception)
{

	return cFilter::Start(__exception_ptr);
}

tResult cClassificationPostprocessing::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}
tResult cClassificationPostprocessing::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
		
		//mediatype für signalvalues
		// tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		// RETURN_IF_POINTER_NULL(strDescSignalValue);
		// cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0,0,0,"tSignalValue",strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		

		//media type für bools
		tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBool);
		cObjectPtr<IMediaType> pTypeSignalBool = new cMediaType(0,0,0,"tBoolSignalValue",strDescBool, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


		//media type fuer tInt32
		tChar const * strDescIntVal = pDescManager->GetMediaDescription("tInt32SignalValue");
		RETURN_IF_POINTER_NULL(strDescIntVal);
		cObjectPtr<IMediaType> pTypeSignalInt = new cMediaType(0,0,0,"tInt32SignalValue",strDescIntVal, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue1));
		
		RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean1));
		RETURN_IF_FAILED(pTypeSignalInt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInt));

		//  Input pins
		

		// Classification Input
		RETURN_IF_FAILED(m_oClassificationPin.Create("classification", new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, 					MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oClassificationPin));
		
		// Output pins
		
		RETURN_IF_FAILED(m_oOutputPersonOnStreetSeen.Create("PersonOnStreetOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputPersonOnStreetSeen));
		//PersonOnStreet

		// Zebrastreifen output
		RETURN_IF_FAILED(m_oOutputZebraStreifen.Create("PersonTowardsStreet",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputZebraStreifen));

		// Crossing output
		RETURN_IF_FAILED(m_oOutputCrossing.Create("CrossingOut",pTypeSignalInt,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputCrossing));


		RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean2));	
	
		// Kid seen output
		RETURN_IF_FAILED(m_oOutputKidSeen.Create("KidSeenOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputKidSeen));
		
		// Server is ready output
		RETURN_IF_FAILED(m_oOutputServerIsRunning.Create("ServerIsRunningOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputServerIsRunning));

		// Brake light output
		RETURN_IF_FAILED(m_oOutputBrakeLight.Create("BrakeLightOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));

	}
	else if (eStage == StageNormal)
	{
	}

	else if (eStage == StageGraphReady)
	{
		
	}

	RETURN_NOERROR;
}



tResult cClassificationPostprocessing::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	if (eStage == StageGraphReady)
	{
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cClassificationPostprocessing::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
        if (pSource == &m_oClassificationPin)
		{

			// If this is the first classification we receive, we have to tell "the brain" that our car is ready to go.
			// Therefore we will send bool over the output pin here.
			TransmitServerRunning(tTrue, GETTIME());

			classificationResults.resize(pMediaSample->GetSize() / sizeof(classificationResults));
			
			//get the date from the media sample
			tVoid* pIncomingData;
			if (IS_OK(pMediaSample->Lock((const tVoid**)&pIncomingData)))
			{
				//make copy
				memcpy(classificationResults.data(), pIncomingData, pMediaSample->GetSize());
				pMediaSample->Unlock(pIncomingData);
			}

			if (!classificationResults.empty())
			{
			   
				CarsNPersonsLocationVector = TrafficLocation(classificationResults,m_filterProperties.LeftSidePercentage,m_filterProperties.RightSidePercentage);
				PersonOnStreetVector = TrafficLocation(classificationResults,m_filterProperties.PersonOnStreetLeftSidePercentage,m_filterProperties.PersonOnStreetRightSidePercentage);
				if(Search4Kids(classificationResults)){

					PRINT("kid seen!");

					TransmitKid(tTrue, GETTIME());
		
				}
				TransmitPersonOnStreet(FindPersonOnStreet(PersonOnStreetVector), GETTIME());

				// Verschicken der aktuellen Situation in Bezug auf Menschen, die auf die Straße laufen
				TransmitZebra(PersonOrientation(CarsNPersonsLocationVector), GETTIME());
				if(m_filterProperties.debugOutput) { 
					LOG_INFO(adtf_util::cString::Format("Person next to street? %i", PersonOrientation(CarsNPersonsLocationVector))); 
				}

				// Verschicken der aktuellen Situation in Bezug auf Autos an einer Kreuzung.
				TransmitCrossing(AnalyseCrossing(CarsNPersonsLocationVector), GETTIME());

			}


		}
	} else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		
	}
	RETURN_NOERROR;
}

// Distance to an object
tFloat32 cClassificationPostprocessing::DistanceToObject(tFloat32 ymax)  
{
    return ( tan((ymax-321.2250165)/204.41201081) + 25.09756939)/(-660.29893851);
}

vector<vector<cnnClassificationResult> > cClassificationPostprocessing::TrafficLocation(std::vector<cnnClassificationResult> resultVector, tFloat32 left_percent, tFloat right_percent)
{
	vector<cnnClassificationResult> left;
	vector<cnnClassificationResult> right;
	vector<cnnClassificationResult> center;
	vector<vector<cnnClassificationResult> > trafficSituation;
	
	trafficSituation.push_back(left);
	trafficSituation.push_back(center);
	trafficSituation.push_back(right);
	
	
	for (std::vector<cnnClassificationResult>::iterator it = resultVector.begin(); it != resultVector.end(); it++)
	{
		
		if((str_comp(it->cnnClassificationDesc, "teddy bear") || str_comp(it->cnnClassificationDesc, "person") || str_comp(it->cnnClassificationDesc, "car")) && it->probability * 100. > m_filterProperties.minProbability   )
		{
			
			if(it->xmin > right_percent)
			{
				trafficSituation[2].push_back(*it);
				if(m_filterProperties.debugOutput) { 
                    //	LOG_INFO(adtf_util::cString::Format("right: %s", it->cnnClassificationDesc)); 
				}
			
			} else if (it -> xmax < left_percent){
				trafficSituation[0].push_back(*it);
				if(m_filterProperties.debugOutput) { 
                    //	LOG_INFO(adtf_util::cString::Format("left: %s", it->cnnClassificationDesc)); 
				}

			} else if (it -> xmin < left_percent && it -> xmax <right_percent){
				trafficSituation[0].push_back(*it);
				trafficSituation[1].push_back(*it);

				if(m_filterProperties.debugOutput) { 
                    //	LOG_INFO(adtf_util::cString::Format("left and center: %s", it->cnnClassificationDesc)); 
				}
				//}
			} else if (it -> xmin > left_percent && it -> xmax <right_percent){

				trafficSituation[1].push_back(*it);
				if(m_filterProperties.debugOutput) { 
                    //	LOG_INFO(adtf_util::cString::Format("center: %s", it->cnnClassificationDesc)); 
				}

			} else if (it -> xmin > left_percent && it -> xmax >right_percent){
				trafficSituation[2].push_back(*it);
				trafficSituation[1].push_back(*it);

				if(m_filterProperties.debugOutput) { 
                    //	LOG_INFO(adtf_util::cString::Format("center and right: %s", it->cnnClassificationDesc)); 
				}
				//}
			} else if (it -> xmin < left_percent && it -> xmax > right_percent){

				trafficSituation[0].push_back(*it);
				trafficSituation[1].push_back(*it);
				trafficSituation[2].push_back(*it);
				if(m_filterProperties.debugOutput) { 
                    //	LOG_INFO(adtf_util::cString::Format("left center right: %s", it->cnnClassificationDesc)); 
				}
			}
		}
	}
	return trafficSituation;
}

tBool cClassificationPostprocessing::FindPersonOnStreet(vector<vector<cnnClassificationResult> > results)
{
	for(std::vector<cnnClassificationResult>::iterator it = results[1].begin(); it != results[1].end(); it++)
	{
		if(str_comp(it->cnnClassificationDesc,"person") && DistanceToObject(it->ymax) < m_filterProperties.PersonOnStreetEmergencyStopDistance)
        {
            return tTrue;
        }	
    }
    return tFalse;	
}

tResult cClassificationPostprocessing::TransmitPersonOnStreet(const tBool personOnStreet, tTimeStamp timestamp)
{

	__synchronized_obj(CritPersonOnStreetOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean2->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean2, pMediaSample, pCoderOutput);

		if (!m_szIDPersonOnStreetSeenSet)
		{
			pCoderOutput->GetID("bValue",m_szIdPersonOnStreetSeen);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDPersonOnStreetSeenTimeStamp);
			m_szIDPersonOnStreetSeenSet = tTrue;
		}
		pCoderOutput->Set(m_szIdPersonOnStreetSeen, (tVoid*)&personOnStreet);
		pCoderOutput->Set(m_szIDPersonOnStreetSeenTimeStamp, (tVoid*)&timestamp);
	

		pMediaSample->SetTime(timestamp);
	}
	RETURN_IF_FAILED(m_oOutputPersonOnStreetSeen.Transmit(pMediaSample));
	RETURN_NOERROR;
}


tResult cClassificationPostprocessing::TransmitKid(const tBool kid, tTimeStamp timestamp)
{

	__synchronized_obj(CritKidOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean2->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean2, pMediaSample, pCoderOutput);

		if (!m_szIDKidSeenSet)
		{
			pCoderOutput->GetID("bValue",m_szIdKidSeen);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDKidSeenTimeStamp);
			m_szIDKidSeenSet = tTrue;
		}
		pCoderOutput->Set(m_szIdKidSeen, (tVoid*)&kid);
		pCoderOutput->Set(m_szIDKidSeenTimeStamp, (tVoid*)&timestamp);
	

		pMediaSample->SetTime(timestamp);
	}
	RETURN_IF_FAILED(m_oOutputKidSeen.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cClassificationPostprocessing::TransmitZebra(const tBool zebra, tTimeStamp timestamp)
{
	__synchronized_obj(CritZebraOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean2->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean2, pMediaSample, pCoderOutput);

		if (!m_szIDZebraStreifenSet)
		{
			pCoderOutput->GetID("bValue",m_szIDZebraStreifen);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDZebraStreifenTimeStamp);
			m_szIDZebraStreifenSet = tTrue;
		}
		pCoderOutput->Set(m_szIDZebraStreifen, (tVoid*)&zebra);
		pCoderOutput->Set(m_szIDZebraStreifenTimeStamp, (tVoid*)&timestamp);
	

		pMediaSample->SetTime(timestamp);
	}
	RETURN_IF_FAILED(m_oOutputZebraStreifen.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cClassificationPostprocessing::TransmitBrakeLight(const tBool brake, tTimeStamp timestamp)
{
	__synchronized_obj(CritBrakeLightOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean2->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean2, pMediaSample, pCoderOutput);

		if (!m_szIDBrakeLightSet)
		{
			pCoderOutput->GetID("bValue",m_szIdBrakeLight);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDBrakeLightTimeStamp);
			m_szIDBrakeLightSet = tTrue;
		}
		pCoderOutput->Set(m_szIdBrakeLight, (tVoid*)&brake);
		pCoderOutput->Set(m_szIDBrakeLightTimeStamp, (tVoid*)&timestamp);
	

		pMediaSample->SetTime(timestamp);
	}
	RETURN_IF_FAILED(m_oOutputBrakeLight.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tResult cClassificationPostprocessing::TransmitServerRunning(const tBool running, tTimeStamp timestamp)
{

	__synchronized_obj(CritServerRunningOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean2->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean2, pMediaSample, pCoderOutput);

		if (!m_szIDServerIsRunningSet)
		{
			pCoderOutput->GetID("bValue",m_szIdServerIsRunning);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDServerIsRunningTimeStamp);
			m_szIDServerIsRunningSet = tTrue;
		}
		pCoderOutput->Set(m_szIdServerIsRunning, (tVoid*)&running);
		pCoderOutput->Set(m_szIDServerIsRunningTimeStamp, (tVoid*)&timestamp);
	

		pMediaSample->SetTime(timestamp);
	}
	RETURN_IF_FAILED(m_oOutputServerIsRunning.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tInt32 cClassificationPostprocessing::AnalyseCrossing(vector<vector<cnnClassificationResult> > CarsAndPersons)
{
	
	// 1 auto
	// hunderter left
	// zehner center
	// einer right
	tInt32 CAR   	 = 1;
	tInt32 LEFT   	 = 100;
	tInt32 CENTER 	 = 10;
	tInt32 RIGHT  	 = 1;	
	tInt32 situation = 0;

	for(int i = 0; i < (int) CarsAndPersons.size();++i)
	{
		switch(i)
		{		
			//left
		case 0 :
			for(std::vector<cnnClassificationResult>::iterator iter = CarsAndPersons[i].begin(); iter != CarsAndPersons[i].end(); iter++)
			{
				if(str_comp(iter->cnnClassificationDesc,"car")){situation += LEFT*CAR;}
				//PRINT("left car");
			}
			break;
   		
			//center		
		case 1 : 
			for(std::vector<cnnClassificationResult>::iterator iter = CarsAndPersons[i].begin(); iter != CarsAndPersons[i].end(); iter++)
			{		
				if(str_comp(iter->cnnClassificationDesc,"car")){situation += CENTER*CAR;}
				//PRINT("center car");
			}		
			break;

			//right
		case 2 : 
			for(std::vector<cnnClassificationResult>::iterator iter = CarsAndPersons[i].begin(); iter != CarsAndPersons[i].end(); iter++)
			{		
				if(str_comp(iter->cnnClassificationDesc,"car")){situation += RIGHT*CAR;}
				//PRINT("right car");
			}		
			break;
		default: 
			break;
		}
	}
	return situation;
}


tResult cClassificationPostprocessing::TransmitCrossing(const tInt32 cars, tTimeStamp timestamp)
{
	
	__synchronized_obj(CritCrossingOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionInt->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionInt, pMediaSample, pCoderOutput);

		if (!m_szIDCrossingSet)
		{
			pCoderOutput->GetID("intValue",m_szIDCrossing);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDCrossingTimeStamp);
			m_szIDCrossingSet = tTrue;
		}
		pCoderOutput->Set(m_szIDCrossing, (tVoid*)&cars);
		pCoderOutput->Set(m_szIDCrossingTimeStamp, (tVoid*)&timestamp);
	

		pMediaSample->SetTime(timestamp);
	}
	RETURN_IF_FAILED(m_oOutputCrossing.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tBool cClassificationPostprocessing::Search4Kids(vector<cnnClassificationResult>  classifications)
{
	tFloat32   sum = 0;
	tFloat32 len = kid_val.size();	
	tFloat32 avrg = 0;	

	for(std::vector<cnnClassificationResult>::iterator it = classifications.begin(); it != classifications.end(); it++)
	{
		bool acceptableProbability =  it->probability *  100.0 > m_filterProperties.minProbability;
		if(acceptableProbability && it->kid && m_filterProperties.kidClassification)
		{
			kid_val.push_back(1.0f);
			for(int i=0; i< len; ++i)
			{
				sum += kid_val[i];
			}	
			avrg = sum/len;
			//PRINT("hallo hier bin ich!");
			//PRINT2("1 = kid: %f  vec len: %f", avrg, len);
			return tTrue;
		}
	}
	kid_val.push_back(0.0f);
	for(int i=0; i< len; ++i)
	{
		sum += kid_val[i];
	}	
	avrg = sum/len;	
	//PRINT2("0 = adult: %f  vec len: %f", avrg, len);
	return tFalse;
}

tBool cClassificationPostprocessing::PersonTowardsStreet(vector<vector<cnnClassificationResult> > results)
{
	for(unsigned int i=0 ; i < results.size(); i++)
	{
		for(std::vector<cnnClassificationResult>::iterator it = results[i].begin(); it != results[i].end(); it++)
		{
			switch(i)
			{
			case 0 :
				if(str_comp(it->cnnClassificationDesc,"person") && str_comp(it->orientation,"right"))
				{
					return tTrue;
				}
				break;

			case 1 : 
				if(str_comp(it->cnnClassificationDesc,"person"))
				{
					return tTrue;
				}	
				break;

			case 2 : 
				if(str_comp(it->cnnClassificationDesc,"person") && str_comp(it->orientation,"left"))
				{
					return tTrue;
				}
				break;

			default: 
				break;
			}
		}
	}
	return tFalse;	
}

tResult cClassificationPostprocessing::PersonOrientation(vector<vector<cnnClassificationResult> > results)
{
	tBool personFoundLeft =  tFalse;
	tBool personFoundMiddle =  tFalse;
	tBool personFoundRight =  tFalse;
	
	for(unsigned int i=0 ; i < results.size(); i++)
	{
		for(std::vector<cnnClassificationResult>::iterator it = results[i].begin(); it != results[i].end(); it++)
		{
			if(!str_comp(it->cnnClassificationDesc,"person")) continue;
			
			switch(i)
			{
			case 0 :
				left.pop_front();
				
				if(str_comp(it->orientation,"right")) {
					left.push_back(1);
				} else if(str_comp(it->orientation,"left")) {
					left.push_back(-1);
				} else {
					left.push_back(0);
				}
					
				personFoundLeft = tTrue;

				break;       // and exits the switch

			case 1 : 
				center.pop_front();
				center.push_back(1);
				personFoundMiddle = tTrue;
				break;

			case 2 : 
				right.pop_front();
					
				if(str_comp(it->orientation,"left")) {
					right.push_back(1);
				} else if(str_comp(it->orientation,"right")) {
					right.push_back(-1);
				} else {
					right.push_back(0);
				}

				personFoundRight = tTrue;
				break;
				
			default: 
				break;
			}
		}
	}

	// If we did not find a person in one of the directions, we have to reset one element of the queue
	if(!personFoundLeft) {
		left.pop_front();
		left.push_back(0);
	}
	
	if(!personFoundMiddle) {
		center.pop_front();
		center.push_back(0);
	}
	
	if(!personFoundRight) {
		right.pop_front();
		right.push_back(0);
	}

	// Reset the sums
	left_sum = 0;
	center_sum = 0;
	right_sum = 0;
	// Add up the new values
	for(int k = 0; k < m_filterProperties.CrossingClassificationQueueSize; ++k)
	{
		left_sum += left[k];
		right_sum += right[k];
		center_sum += center[k];
	}
	
	int threshold = m_filterProperties.personOrientationThreshold; 
	if(left_sum > threshold || right_sum > threshold || center_sum > threshold)
	{
		if(m_filterProperties.debugOutput) { 
			//LOG_INFO(cString::Format("Person towards street. left: %i,  center: %i, right: %i", left_sum, center_sum, right_sum)); 
		}
		return tTrue;
	}
	return tFalse;	

}
