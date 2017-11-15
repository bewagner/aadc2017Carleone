/**
   Copyright (c)
   Audi Autonomous Driving Cup. All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
   4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   **********************************************************************
   * $Author:: spiesra $  $Date:: 2017-04-28 08:10:05#$ $Rev:: 62659   $
   **********************************************************************/
#include "stdafx.h"
#include "cClassificationPictureCapturer.h"
#include <iostream>
#include <string>
#include <ctime>
#include <math.h>
#include <sstream>
#include <stdlib.h>

// define the ADTF property names to avoid errors 
ADTF_FILTER_PLUGIN(ADTF_BFFT_FILTER_DESC,
				   OID_ADTF_BFFT_FILTER_DEF,
				   cClassificationPictureCapturer)



cClassificationPictureCapturer::cClassificationPictureCapturer(const tChar* __info) : cFilter(__info)
{    
	lastClassificationResultsSet = tFalse;		
}

cClassificationPictureCapturer::~cClassificationPictureCapturer()
{
	
}

tResult cClassificationPictureCapturer::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cClassificationPictureCapturer::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cClassificationPictureCapturer::PropertyChanged(const tChar* strName)
{

	RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
	
	RETURN_NOERROR;
}





tResult cClassificationPictureCapturer::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
		
        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
		
		//create pin for input
		RETURN_IF_FAILED(m_oClassificationPin.Create("classification", new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oClassificationPin));

		tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBool);
		cObjectPtr<IMediaType> pTypeSignalBool = new cMediaType(0,0,0,"tBoolSignalValue",strDescBool, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean));

		RETURN_IF_FAILED(m_oInputBool.Create("TakePicture",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputBool));


    }
    else if (eStage == StageNormal)
    {
		
    }

    else if (eStage == StageGraphReady)
    {
		// get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("Invalid Input Format for this filter");
        }
    }

    RETURN_NOERROR;
}



tResult cClassificationPictureCapturer::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cClassificationPictureCapturer::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	// first check what kind of event it is


	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		RETURN_IF_POINTER_NULL(pMediaSample);
		RETURN_IF_POINTER_NULL(pSource);
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
				// Assign the result of the classification to our local variable
				lastClassificationResults =  classificationResults;

				// Set this bool, so we know, we assigned a classification result
				if(!lastClassificationResultsSet)  {
					lastClassificationResultsSet = tTrue;
}
			}
		}
		else if (pSource == &m_oVideoInputPin)
		{
			//check if video format is still unkown
			if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
			{
				RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
			}
			ProcessVideo(pMediaSample);
			
		} else if(pSource == &m_oInputBool) {
			TakePicture();
		}
		
	}
	else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		if (pSource == &m_oVideoInputPin)
		{
			//the input format was changed, so the imageformat has to changed in this filter also
			RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
		}
	}
	RETURN_NOERROR;

}

tResult cClassificationPictureCapturer::ProcessVideo(IMediaSample* pSample)
{
	RETURN_IF_POINTER_NULL(pSample);
	const tVoid* l_pSrcBuffer;

	//receiving data from input sample, and saving to TheInputImage
	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{
		//convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
		{
			
			m_inputImage.data = (uchar*)(l_pSrcBuffer);			
			
		}
		pSample->Unlock(l_pSrcBuffer);
	}
	
    
	RETURN_NOERROR;
}

tResult cClassificationPictureCapturer::TakePicture()
{

	if(!lastClassificationResults.empty()) {

		// Go through the vector of classification results and search for results labeled as the given search label
		string searchLabel ("person");
				
		// The directory to save the pictures in
		string baseDir ("/home/aadc/ADTF/src/aadcUser/pictures/");
				
		int count =  0;
				
		for (std::vector<cnnClassificationResult>::iterator it = lastClassificationResults.begin(); it != lastClassificationResults.end(); it++){
			string currentLabel (it->cnnClassificationDesc);
					
			// Search for classifications that match our search label and have a minimum probability of 70 %
			if(searchLabel.compare(currentLabel) == 0 && it->probability > 0.50) {
						
				// Since the classification gives us percentage values for xmin, ymin, ymax and xmax, here we calculate the real values.
				tInt cutOutXMin = floor(m_inputImage.cols * it->xmin);
				tInt cutOutYMin = floor(m_inputImage.rows *  it->ymin);
				tInt cutOutWidth = floor(m_inputImage.cols *  (it->xmax - it->xmin));
				tInt cutOutHeight = floor(m_inputImage.rows *  (it->ymax - it->ymin));
				cv::Rect currentRectangle = Rect(cutOutXMin, cutOutYMin, cutOutWidth, cutOutHeight );
				// Safety checks
				if(cutOutXMin > 0 && cutOutXMin < m_inputImage.cols
				   && cutOutYMin > 0 && cutOutYMin < m_inputImage.rows
				   && cutOutWidth > 0 && cutOutXMin +  cutOutWidth < m_inputImage.cols
				   && cutOutHeight > 0 && cutOutYMin + cutOutHeight < m_inputImage.rows) {

					// The next few lines are just for getting the current time and formatting it as a string.
					time_t rawtime;
					struct tm * timeinfo;
					char buffer[80];

					time (&rawtime);
					timeinfo = localtime(&rawtime);

					strftime(buffer,sizeof(buffer),"%d-%m-%Y_%I-%M-%S",timeinfo);
					std::string dateAndTime(buffer);							

					
					stringstream ss;
					ss << count;
					string countString = ss.str();
					
					cv::Mat croppedRef(m_inputImage, currentRectangle);
					cv::imwrite(baseDir + searchLabel + countString+ "_" + dateAndTime + ".jpg", croppedRef );	
				}
			}
		}
	}
	RETURN_NOERROR;
}




tResult cClassificationPictureCapturer::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		//update member variable
		m_sInputFormat = (*pFormat);
		LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
		//create the input matrix
		RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
	}
	RETURN_NOERROR;
}
