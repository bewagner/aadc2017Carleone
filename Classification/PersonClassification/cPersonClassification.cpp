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
#include "stdafx.h"
#include "cPersonClassification.h"
//#include "aadc_classification_structs.h"
#include <iostream>
#include <string>




// define the ADTF property names to avoid errors 
ADTF_FILTER_PLUGIN(ADTF_BFFT_FILTER_DESC,
				   OID_ADTF_BFFT_FILTER_DEF,
				   cPersonClassification)



cPersonClassification::cPersonClassification(const tChar* __info) : cFilter(__info)
{    
	SetPropertyFloat("FILTERING::MinProbability", 75.0);
	SetPropertyStr("FILTERING::MinProbability" NSSUBPROP_DESCRIPTION, "The Minimum probability for a classified object to be displayed in the output picture.");
	SetPropertyBool("FILTERING::MinProbability" NSSUBPROP_ISCHANGEABLE, tTrue);
}

cPersonClassification::~cPersonClassification()
{
}

tResult cPersonClassification::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cPersonClassification::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cPersonClassification::PropertyChanged(const tChar* strName)
{

	RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
	//associate the properties to the member
	if (cString::IsEqual(strName, "FILTERING::MinProbability"))
		m_filterProperties.minProbability = GetPropertyFloat("FILTERING::MinProbability");

	RETURN_NOERROR;
}





tResult cPersonClassification::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		//create pin for input
		RETURN_IF_FAILED(m_oClassificationPin.Create("classification", new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oClassificationPin));

		// GCL Output
		RETURN_IF_FAILED(m_oGCLOutputPin.Create("GCL", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oGCLOutputPin)); 

		//create pin for RawDepthData
		RETURN_IF_FAILED(m_inputDepthRaw.Create("DepthData", new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_inputDepthRaw));

		// GCL Output
		RETURN_IF_FAILED(m_oClassificationBool.Create("ClassificationBool", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oClassificationBool)); 

    }
    else if (eStage == StageNormal)
    {
    }

    else if (eStage == StageGraphReady)
    {
 
    }

    RETURN_NOERROR;
}



tResult cPersonClassification::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cPersonClassification::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		RETURN_IF_POINTER_NULL(pMediaSample);
		if (pSource == &m_oClassificationPin)
		{
			std::vector<cnnClassificationResult> classificationResults;
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
				// Transmit the detected objects to the output window
				transmitClassification(classificationResults);
			}
		}
	}
	else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
	}

	RETURN_NOERROR;
}

tResult cPersonClassification::transmitClassification(std::vector<cnnClassificationResult> resultVector)
{
	tFloat64 segregationPercent = 0.541;			
	
	//set color
		for (std::vector<cnnClassificationResult>::iterator it = resultVector.begin(); it != resultVector.end(); it++)
		{
		//cout << "hello" << endl;
			if(((string)it->cnnClassificationDesc).compare("person") && it->ymin != 0.&& it->probability * 100 > m_filterProperties.minProbability && it-> ymin < segregationPercent){
			//cout << "it's a woman" << endl;
			}
					
		}

		RETURN_NOERROR;
}

/*
// Transmit Function for Raw Depth Data
tResult cDepthVisuSlam::TransmitDepthRaw(const void *pData)
{
    //Creating new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    //allocing buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(DEPTH_FRAME_SIZE));
    //updating media sample
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), pData, DEPTH_FRAME_SIZE, IMediaSample::MSF_None));
    //transmitting
    RETURN_IF_FAILED(m_outputDepthRaw.Transmit(pMediaSample));

    RETURN_NOERROR;
}*/
