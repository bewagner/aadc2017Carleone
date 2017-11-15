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
   * $Author:: spiesra $  $Date:: 2017-05-22 18:08:00#$ $Rev:: 63774   $
   **********************************************************************/
#include "stdafx.h"
#include "StopLineDetector.h"
#include <iostream>
#include "functions.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("StopLineDetector","StopLineDetector",StopLineDetector)



StopLineDetector::StopLineDetector(const tChar* __info) : cFilter(__info)
{
	SetPropertyBool("DEBUG::DebutOutput", tFalse);
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("DEBUG::DebutOutput", tFalse);
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("DEBUG::DebutOutput", tFalse);
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	SendLock = tFalse;

	SetPropertyInt("ROI::XOffset", 540);
	SetPropertyStr("ROI::XOffset" NSSUBPROP_DESCRIPTION, "X Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::XOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::YOffset", 590);
	SetPropertyStr("ROI::YOffset" NSSUBPROP_DESCRIPTION, "Y Offset for RIght Region of Interest Rectangular");
	SetPropertyBool("ROI::YOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::Width", 200);
	SetPropertyStr("ROI::Width" NSSUBPROP_DESCRIPTION, "Width of the Region of Interest Rectangular");
	SetPropertyBool("ROI::Width" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::Height", 25);
	SetPropertyStr("ROI::Height" NSSUBPROP_DESCRIPTION, "Height of the Region of Interest Rectangular");
	SetPropertyBool("ROI::Height" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("ROI::StraightnessThreshold", 30);
	SetPropertyStr("ROI::StraightnessThreshold" NSSUBPROP_DESCRIPTION, "Threshold of StopLineDetection Straightness");
	SetPropertyBool("ROI::StraightnessThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("ROI::DynamicMultiplier", 10);
	SetPropertyStr("ROI::DynamicMultiplier" NSSUBPROP_DESCRIPTION, "DynamicMultiplier");
	SetPropertyBool("ROI::DynamicMultiplier" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::MaxDynamicWidth", 100);
	SetPropertyStr("ROI::MaxDynamicWidth" NSSUBPROP_DESCRIPTION, "MaxDynamicWidth");
	SetPropertyBool("ROI::MaxDynamicWidth" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointOneX", 100);
	SetPropertyStr("ROITWO::PointOneX" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROITWO::PointOneX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointTwoX", 150);
	SetPropertyStr("ROITWO::PointTwoX" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROITWO::PointTwoX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointOneY", 100);
	SetPropertyStr("ROITWO::PointOneY" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROITWO::PointOneY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointTwoY", 150);
	SetPropertyStr("ROITWO::PointTwoY" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROITWO::PointTwoY" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyInt("Algorithm::Detection Lines", 10);
	SetPropertyStr("Algorithm::Detection Lines" NSSUBPROP_DESCRIPTION, "number of detection lines searched in ROI");
	SetPropertyBool("Algorithm::Detection Lines" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Detection Lines" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Minimum Line Width", 10);
	SetPropertyStr("Algorithm::Minimum Line Width" NSSUBPROP_DESCRIPTION, "Minimum Line Width in Pixel");
	SetPropertyBool("Algorithm::Minimum Line Width" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Minimum Line Width" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Maximum Line Width", 30);
	SetPropertyStr("Algorithm::Maximum Line Width" NSSUBPROP_DESCRIPTION, "Maximum Line Width in Pixel");
	SetPropertyBool("Algorithm::Maximum Line Width" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Maximum Line Width" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Minimum Line Contrast", 50);
	SetPropertyStr("Algorithm::Minimum Line Contrast" NSSUBPROP_DESCRIPTION, "Mimimum line contrast in gray Values");
	SetPropertyBool("Algorithm::Minimum Line Contrast" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MIN, 1);
	SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MAX, 255);

	SetPropertyInt("Algorithm::Image Binarization Threshold", 180);
	SetPropertyStr("Algorithm::Image Binarization Threshold" NSSUBPROP_DESCRIPTION, "Threshold for image binarization");
	SetPropertyBool("Algorithm::Image Binarization Threshold" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MIN, 1);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MAX, 255);


	SetPropertyFloat("Canny::Threshold", 100.0f);
	SetPropertyStr("Canny::Threshold" NSSUBPROP_DESCRIPTION, "Threshold of Edge Detection");
	SetPropertyBool("Canny::Threshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Canny::ThresholdRatio", 1.5f);
	SetPropertyStr("Canny::ThresholdRatio" NSSUBPROP_DESCRIPTION, "Ratio of Edge Detection Threshold");
	SetPropertyBool("Canny::ThresholdRatio" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Canny::KernelSize", 3.0f);
	SetPropertyStr("Canny::KernelSize" NSSUBPROP_DESCRIPTION, "KernelSize of Edge Detection");
	SetPropertyBool("Canny::KernelSize" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Hough::AccumulatorThreshold", 100.0f);
	SetPropertyStr("Hough::AccumulatorThreshold" NSSUBPROP_DESCRIPTION, "Amount of Intersections needed to detect a line");
	SetPropertyBool("Hough::AccumulatorThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Hough::AngleDiameter", 0.05f);
	SetPropertyStr("Hough::AngleDiameter" NSSUBPROP_DESCRIPTION, "Amount of Variation allowed to constitue a horizontal line in rad");
	SetPropertyBool("Hough::AngleDiameter" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyFloat("Distance::LowDistance", 0.5f);
	SetPropertyStr("Distance::LowDistance" NSSUBPROP_DESCRIPTION, "Lowest possible Distance to Line");
	SetPropertyBool("Distance::LowDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Distance::HighDistance", 0.8f);
	SetPropertyStr("Distance::HighDistance" NSSUBPROP_DESCRIPTION, "Highest possible Distance to Line");
	SetPropertyBool("Distance::HighDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Corners::QualityMeasure", 0.04f);
	SetPropertyStr("Corners::QualityMeasure" NSSUBPROP_DESCRIPTION, "QualityMeasure");
	SetPropertyBool("Corners::QualityMeasure" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Corners::MaxDistance", 0.04f);
	SetPropertyStr("Corners::MaxDistance" NSSUBPROP_DESCRIPTION, "MaxDistance");
	SetPropertyBool("Corners::MaxDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Corners::NumberOfCorners", 5);
	SetPropertyStr("Corners::NumberOfCorners" NSSUBPROP_DESCRIPTION, "NumberOfCorners");
	SetPropertyBool("Corners::NumberOfCorners" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Corners::PropCorrFactor", 0.0f);
	SetPropertyStr("Corners::PropCorrFactor" NSSUBPROP_DESCRIPTION, "PropCorrFactor");
	SetPropertyBool("Corners::PropCorrFactor" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Corners::LateralCorrFactor", 0.0f);
	SetPropertyStr("Corners::LateralCorrFactor" NSSUBPROP_DESCRIPTION, "LateralCorrFactor");
	SetPropertyBool("Corners::LateralCorrFactor" NSSUBPROP_ISCHANGEABLE, tTrue);

    HorizontalLineDetected = tFalse;
    m_DistanceOutSet = tFalse;
    m_OvertakingPossibleOutSet = tFalse;
    m_szIDsOutputBoolSet = tFalse;
    m_SteeringBufferSet = tFalse;
    m_SpeedBufferSet = tFalse;
    Speed = 0.0f;
    SteeringValue = 0.0f;
    Offset = 0;

}

StopLineDetector::~StopLineDetector()
{
}

tResult StopLineDetector::Start(__exception)
{

	return cFilter::Start(__exception_ptr);
}

tResult StopLineDetector::Stop(__exception)
{
	//destroyWindow("Debug");
	return cFilter::Stop(__exception_ptr);
}
tResult StopLineDetector::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{

		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

		RETURN_IF_FAILED(m_oVideoInputPinTwo.Create("Video_Input_Two", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPinTwo));

		// Video Output
		RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output_Debug", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));
		// Video Output
		RETURN_IF_FAILED(m_oVideoOutputPinTwo.Create("Video_Output_Two_Debug", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPinTwo));

		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
		//get description
		tChar const * strDescignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);


		tChar const * strDescTSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		cObjectPtr<IMediaType> pTypeTSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescTSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		// checks if exists
		RETURN_IF_POINTER_NULL(strDescignalValue);
		//RETURN_IF_POINTER_NULL(strDescTSignalValue);

		//get mediatype

		//

		//get mediatype description
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
		RETURN_IF_FAILED(pTypeTSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

		//create pin for output
		RETURN_IF_FAILED(m_oBoolValuePin.Create("BoolValue",pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oBoolValuePin));
        
		RETURN_IF_FAILED(m_OvertakingPossibleOutPin.Create("OvertakingPossible",pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_OvertakingPossibleOutPin));

		RETURN_IF_FAILED(m_DistanceOut.Create("DistanceOut",pTypeTSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_DistanceOut));

		RETURN_IF_FAILED(m_SteeringInput.Create("SteeringIn", pTypeTSignalValue,static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_SteeringInput));

		RETURN_IF_FAILED(m_SpeedIn.Create("SpeedIn", pTypeTSignalValue,static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_SpeedIn));


	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageGraphReady)
	{
		// get the image format of the input video pin
		cObjectPtr<IMediaType> pType;
		cObjectPtr<IMediaType> pTypeTwo;
		RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
		RETURN_IF_FAILED(m_oVideoInputPinTwo.GetMediaType(&pTypeTwo));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		cObjectPtr<IMediaTypeVideo> pTypeVideoTwo;

		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
		RETURN_IF_FAILED(pTypeTwo->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideoTwo));

		// set the image format of the input video pin
		if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
		{
			LOG_ERROR("Invalid Input Format for this filter");
		}
		if (IS_FAILED(UpdateInputImageFormatTwo(pTypeVideoTwo->GetFormat())))
		{
			LOG_ERROR("Invalid Input Format for this filter");
		}
	}

	RETURN_NOERROR;
}



tResult StopLineDetector::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	if (eStage == StageGraphReady)
	{
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult StopLineDetector::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	tTimeStamp inputTimestamp = pMediaSample->GetTime();

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pSource == &m_oVideoInputPin)
		{
			if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
			{
				RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
			}

			RETURN_IF_FAILED(ProcessVideo(pMediaSample, inputTimestamp));
		}
		else if (pSource == &m_oVideoInputPinTwo)
		{
			if (m_sInputFormatTwo.nPixelFormat == IImage::PF_UNKNOWN)
			{
				RETURN_IF_FAILED(UpdateInputImageFormatTwo(m_oVideoInputPinTwo.GetFormat()));
			}

			RETURN_IF_FAILED(ProcessVideoTwo(pMediaSample, inputTimestamp));
		}
		else if (pSource == &m_SteeringInput)
		{
			RETURN_IF_FAILED(ProcessSteering(pMediaSample));
		}
		else if (pSource == &m_SpeedIn)
		{
			RETURN_IF_FAILED(ProcessSpeed(pMediaSample));
		}


	}
	else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		if (pSource == &m_oVideoInputPin)
		{
			//the input format was changed, so the imageformat has to changed in this filter also
			RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
		}
		else if (pSource == &m_oVideoInputPinTwo)
		{
			//the input format was changed, so the imageformat has to changed in this filter also
			RETURN_IF_FAILED(UpdateInputImageFormatTwo(m_oVideoInputPinTwo.GetFormat()));
		}

	}

	RETURN_NOERROR;
}

tResult StopLineDetector::PropertyChanged(const tChar* strName)
{
	RETURN_IF_FAILED(cFilter::PropertyChanged(strName)); 
	if (cString::IsEqual(strName, "DEBUG::DebutOutput"))
	{
		m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");
	}

	if (cString::IsEqual(strName, "DEBUG::DebutOutput"))
	{
		m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");
	}

	if (cString::IsEqual(strName, "DEBUG::DebutOutput"))
	{
		m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");
	}

	//associate the properties to the member
	if (cString::IsEqual(strName, "ROI::Width"))
		m_filterProperties.ROIWidth = GetPropertyInt("ROI::Width");
	else if (cString::IsEqual(strName, "ROI::Height"))
		m_filterProperties.ROIHeight = GetPropertyInt("ROI::Height");
	else if (cString::IsEqual(strName, "ROI::XOffset"))
		m_filterProperties.ROIOffsetX = GetPropertyInt("ROI::XOffset");
	else if (cString::IsEqual(strName, "ROI::StraightnessThreshold"))
		m_filterProperties.StraightnessThreshold = GetPropertyInt("ROI::StraightnessThreshold");
	else if (cString::IsEqual(strName, "ROI::YOffset"))
		m_filterProperties.ROIOffsetY = GetPropertyInt("ROI::YOffset");
	else if (cString::IsEqual(strName, "Algorithm::Detection Lines"))
		m_filterProperties.detectionLines = GetPropertyInt("Algorithm::Detection Lines");
	else if (cString::IsEqual(strName, "Algorithm::Maximum Line Width"))
		m_filterProperties.maxLineWidth = GetPropertyInt("Algorithm::Maximum Line Width");
	else if (cString::IsEqual(strName, "Algorithm::Minimum Line Width"))
		m_filterProperties.minLineWidth = GetPropertyInt("Algorithm::Minimum Line Width");
	else if (cString::IsEqual(strName, "Algorithm::Minimum Line Contrast"))
		m_filterProperties.minLineContrast = GetPropertyInt("Algorithm::Minimum Line Contrast");
	else if (cString::IsEqual(strName, "Algorithm::Image Binarization Threshold"))
		m_filterProperties.thresholdImageBinarization = GetPropertyInt("Algorithm::Image Binarization Threshold");

	else if (cString::IsEqual(strName, "Canny::Threshold"))
		m_filterProperties.Threshold = GetPropertyFloat("Canny::Threshold");
	else if (cString::IsEqual(strName, "Canny::ThresholdRatio"))
		m_filterProperties.ThresholdRatio = GetPropertyFloat("Canny::ThresholdRatio");
	else if (cString::IsEqual(strName, "Canny::KernelSize"))
		m_filterProperties.KernelSize = GetPropertyFloat("Canny::KernelSize");
	else if (cString::IsEqual(strName, "Hough::AccumulatorThreshold"))
		m_filterProperties.AccumulatorThreshold = GetPropertyFloat("Hough::AccumulatorThreshold");
	else if (cString::IsEqual(strName, "Hough::AngleDiameter"))
		m_filterProperties.AngleDiameter = GetPropertyFloat("Hough::AngleDiameter");
	else if (cString::IsEqual(strName, "Hough::AngleDiameter"))
		m_filterProperties.AngleDiameter = GetPropertyFloat("Hough::AngleDiameter");
	else if (cString::IsEqual(strName, "Distance::HighDistance"))
		m_filterProperties.HighDistance = GetPropertyFloat("Distance::HighDistance");
	else if (cString::IsEqual(strName, "Distance::LowDistance"))
		m_filterProperties.LowDistance = GetPropertyFloat("Distance::LowDistance");
	else if (cString::IsEqual(strName, "Corners::QualityMeasure"))
		m_filterProperties.qualityMeasure = GetPropertyFloat("Corners::QualityMeasure");
	else if (cString::IsEqual(strName, "Corners::PropCorrFactor"))
		m_filterProperties.PropCorrectionFactor = GetPropertyFloat("Corners::PropCorrFactor");
	else if (cString::IsEqual(strName, "Corners::MaxDistance"))
		m_filterProperties.distanceOfCorners = GetPropertyFloat("Corners::MaxDistance");
	else if (cString::IsEqual(strName, "Corners::LateralCorrFactor"))
		m_filterProperties.LateralCorrectionFactor = GetPropertyFloat("Corners::LateralCorrFactor");
	else if (cString::IsEqual(strName, "Corners::NumberOfCorners"))
		m_filterProperties.numberOfCorners = GetPropertyInt("Corners::NumberOfCorners");
	else if (cString::IsEqual(strName, "ROI::DynamicMultiplier"))
		m_filterProperties.ROIDynamicMultiplier = GetPropertyInt("ROI::DynamicMultiplier");
	else if (cString::IsEqual(strName, "ROI::MaxDynamicWidth"))
		m_filterProperties.ROIMaxOffset = GetPropertyInt("ROI::MaxDynamicWidth");
	else if (cString::IsEqual(strName, "ROITWO::PointOneX"))
		m_filterProperties.ROITwoP1X = GetPropertyInt("ROITWO::PointOneX");
	else if (cString::IsEqual(strName, "ROITWO::PointTwoX"))
		m_filterProperties.ROITwoP2X = GetPropertyInt("ROITWO::PointTwoX");
	else if (cString::IsEqual(strName, "ROITWO::PointOneY"))
		m_filterProperties.ROITwoP1Y = GetPropertyInt("ROITWO::PointOneY");
	else if (cString::IsEqual(strName, "ROITWO::PointTwoY"))
		m_filterProperties.ROITwoP2Y = GetPropertyInt("ROITWO::PointTwoY");


	RETURN_NOERROR;
}

tResult StopLineDetector::ProcessSteering(IMediaSample* pSample)
{
	__synchronized_obj(critSteeringIn);

	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pSample, pCoderInput);
		tTimeStamp ts;
		if(!m_SteeringBufferSet)
		{
			pCoderInput->GetID("f32Value",m_SteeringBuffer);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_SteeringBufferTs);
			m_SteeringBufferSet = tTrue;
		}
		RETURN_IF_FAILED(pCoderInput->Get(m_SteeringBuffer, (tVoid*)&SteeringValue));
		RETURN_IF_FAILED(pCoderInput->Get(m_SteeringBufferTs, (tVoid*)&ts));
	}
	RETURN_NOERROR;
}

tResult StopLineDetector::ProcessSpeed(IMediaSample* pSample)
{

	__synchronized_obj(critSpeedIn);
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pSample, pCoderInput);
		tTimeStamp ts;
		if(!m_SpeedBufferSet)
		{
			pCoderInput->GetID("f32Value",m_SpeedInBuffer);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_SpeedInBufferTs);
			m_SpeedBufferSet = tTrue;
		}
		RETURN_IF_FAILED(pCoderInput->Get(m_SpeedInBuffer, (tVoid*)&Speed));
		RETURN_IF_FAILED(pCoderInput->Get(m_SpeedInBufferTs, (tVoid*)&ts));
	}
	RETURN_NOERROR;
}
tResult StopLineDetector::ProcessVideoTwo(IMediaSample* pSample, tTimeStamp inputTimestamp)
{
	__synchronized_obj(critVidInTwo);
	RETURN_IF_POINTER_NULL(pSample);
	// new image for result
	cv::Mat outputImage;
	const tVoid* l_pSrcBufferTwo;
	if (IS_OK(pSample->Lock(&l_pSrcBufferTwo)))
	{
		//convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImageTwo.total() * m_inputImageTwo.elemSize()) == m_sInputFormatTwo.nSize)
		{
			m_inputImageTwo.data = (uchar*)(l_pSrcBufferTwo);
		}
		//m_inputImageTwo = Mat(m_sInputFormatTwo.nHeight, m_sInputFormatTwo.nWidth, CV_8UC3, (tVoid*)l_pSrcBuffer, m_sInputFormatTwo.nBytesPerLine);

		cvtColor(m_inputImageTwo, outputImage, CV_RGB2GRAY);
		int width = abs(m_filterProperties.ROITwoP1X - m_filterProperties.ROITwoP2X);
		int height = abs(m_filterProperties.ROITwoP1Y - m_filterProperties.ROITwoP2Y);
		cv::Rect RoiImage = Rect(m_filterProperties.ROITwoP1X,m_filterProperties.ROITwoP1Y,width,height);
		cv::Mat croppedRef(outputImage, RoiImage);
		Canny(croppedRef, croppedRef, m_filterProperties.Threshold, m_filterProperties.ThresholdRatio, m_filterProperties.KernelSize);
		vector<Vec2f> lines;
		HoughLines(croppedRef, lines, 3, CV_PI/180, m_filterProperties.AccumulatorThreshold, 0, 0 );
		//cvtColor(outputImage, outputImage, CV_GRAY2RGB);
		//cvtColor(croppedRef, croppedRef, CV_GRAY2RGB);
		for( size_t i = 0; i < lines.size(); i++ )
		{
			float rho = lines[i][0], theta = lines[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line(croppedRef , pt1, pt2, Scalar(255,0,0), 1, CV_AA);
            if (Betrag(CV_PI/2.0f - theta) < m_filterProperties.AngleDiameter)
            {
                RETURN_IF_FAILED(TransmitStopline(tTrue,GetTime()));
                break;
            }
		}


		pSample->Unlock(l_pSrcBufferTwo);
	}

	if (!outputImage.empty() && m_oVideoOutputPinTwo.IsConnected())
	{
		//PRINT("VideoOut2");
		//__synchronized_obj(critVidOutTwo);
		UpdateOutputImageFormatTwo(outputImage);

		//create a cImage from CV Matrix
		cImage newImage;
		newImage.Create(m_sOutputFormatTwo.nWidth, m_sOutputFormatTwo.nHeight, m_sOutputFormatTwo.nBitsPerPixel, m_sOutputFormatTwo.nBytesPerLine, outputImage.data);

		//create the new media sample
		cObjectPtr<IMediaSample> pMediaSample;
		//RETURN_IF_POINTER_NULL(pMediaSample);
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
		//updating media sample
		RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
		//transmitting
		RETURN_IF_FAILED(m_oVideoOutputPinTwo.Transmit(pMediaSample));

		outputImage.release();
	}
	RETURN_NOERROR;


}
tResult StopLineDetector::ProcessVideo(IMediaSample* pSample, tTimeStamp inputTimestamp)
{
	__synchronized_obj(critVidIn);
	vector<Point2f> corners;

	RETURN_IF_POINTER_NULL(pSample);
	// new image for result
	cv::Mat outputImage;
    
    
    // here we store the pixel lines in the image where we search for lanes

    //vector<tInt> detectionLines;
    
    // here we have all the detected line points

    //vector<cPoint> detectedLinePoints;


    const tVoid* l_pSrcBuffer;

	//receiving data from input sample, and saving to TheInputImage
	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{
		//convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
		{
			m_inputImage.data = (uchar*)(l_pSrcBuffer);
		}
        //imwrite( "../../images/Gray_Image.jpg", m_inputImage );
        Offset = (tInt)(m_filterProperties.ROIDynamicMultiplier*SteeringValue);
        if(abs(Offset) > m_filterProperties.ROIMaxOffset)
        {
            Offset = Offset > 0 ? m_filterProperties.ROIMaxOffset : -m_filterProperties.ROIMaxOffset;
        }
        
        //Using OpenCV
        cvtColor(m_inputImage, outputImage, CV_RGB2GRAY);
        cv::Rect overtakingROI = Rect(200, 550, 100, 100);
        cv::Mat overtakingCrop(outputImage, overtakingROI);
        
        // (..., ..., Threshold, ThresholdRatio, KernelSize)
        Canny(overtakingCrop, overtakingCrop, 100, 200, 3);
        vector<Vec2f> overtakingLines;
        HoughLines(overtakingCrop, overtakingLines, 3, CV_PI/180, 80, 0, 0 );
        cvtColor(overtakingCrop, overtakingCrop, COLOR_GRAY2BGR);
        
        tBool transmitted = tFalse;
        for( size_t i = 0; i < overtakingLines.size(); i++ )
        {
            float theta = overtakingLines[i][1];
            if (Betrag(1.256 - theta) < 0.3)
            {
                RETURN_IF_FAILED(TransmitOvertakingPossible(tTrue));
                transmitted = tTrue;
                break;
            }
        }

        if(!transmitted) {
            RETURN_IF_FAILED(TransmitOvertakingPossible(tFalse));
        }

        cv::Rect RoiImage = Rect(m_filterProperties.ROIOffsetX + Offset,m_filterProperties.ROIOffsetY,m_filterProperties.ROIWidth,m_filterProperties.ROIHeight);
        cv::Mat croppedRef(outputImage, RoiImage);

        
        Mat cornerImage;
        croppedRef.copyTo(cornerImage);
        
        //Canny(croppedRef, croppedRef, m_filterProperties.Threshold, m_filterProperties.ThresholdRatio, m_filterProperties.KernelSize);
        vector<Vec2f> lines;
        Mat mask;
        //mask =Scalar(0);
        //mask(mask,RoiImage) = Scalar(1);
        tInt cornerX = 0;
        goodFeaturesToTrack(cornerImage, corners, m_filterProperties.numberOfCorners, m_filterProperties.qualityMeasure, m_filterProperties.distanceOfCorners, mask, 3, false, 0.04);
        cvtColor(outputImage, outputImage, COLOR_GRAY2BGR);

        if (!corners.empty())
        {
            tInt lowestY = corners.begin()->y; // lowestY ist der im Bild "höchste!" Eckenwert.
            for( size_t i = 0; i < corners.size(); i++ )
            {
                if (corners.at(i).y < lowestY)
                {
                    lowestY = corners.at(i).y;
                    cornerX = corners.at(i).x;
                }
                corners.at(i).x += m_filterProperties.ROIOffsetX + Offset;;
                corners.at(i).y += m_filterProperties.ROIOffsetY;
                //cout << "(" << corners.at(i).x << "," << corners.at(i).y << ")" << endl;
                circle(outputImage,corners.at(i),3, Scalar(255,0,0),5);

            }
            tFloat32 distance = m_filterProperties.LowDistance + (1 - lowestY/(tFloat32)m_filterProperties.ROIHeight)*(m_filterProperties.HighDistance - m_filterProperties.LowDistance);
            distance -= m_filterProperties.PropCorrectionFactor*Speed - m_filterProperties.LateralCorrectionFactor*SteeringValue;
            TransmitStopDistance(distance);
            //TransmitStopline(tTrue,GetTime());
        }
        pSample->Unlock(l_pSrcBuffer);
    }




    if (!outputImage.empty() && m_oVideoOutputPin.IsConnected())
    {
        //PRINT("VideoOutOne");
        //__synchronized_obj(critVidOutTwo);
        UpdateOutputImageFormat(outputImage);

        //create a cImage from CV Matrix
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        //RETURN_IF_POINTER_NULL(pMediaSample);
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

        outputImage.release();
    }

    RETURN_NOERROR;
}


tResult StopLineDetector::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormat = (*pFormat);
        if(m_filterProperties.debugOutput) {
            if(m_filterProperties.debugOutput) {
                if(m_filterProperties.debugOutput) {
                    LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
                }
            }
        }
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
    }
    RETURN_NOERROR;
}
tResult StopLineDetector::UpdateInputImageFormatTwo(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormatTwo = (*pFormat);
        if(m_filterProperties.debugOutput) {
            if(m_filterProperties.debugOutput) {
                if(m_filterProperties.debugOutput) {
                    LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormatTwo.nWidth, m_sInputFormatTwo.nHeight, m_sInputFormatTwo.nBytesPerLine, m_sInputFormatTwo.nSize, m_sInputFormatTwo.nPixelFormat));
                }
            }
        }
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormatTwo, m_inputImageTwo));
    }
    RETURN_NOERROR;
}


tResult StopLineDetector::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        if(m_filterProperties.debugOutput) {
            if(m_filterProperties.debugOutput) {
                if(m_filterProperties.debugOutput) {
                    LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
                }
            }
        }
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult StopLineDetector::UpdateOutputImageFormatTwo(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormatTwo.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormatTwo);

        if(m_filterProperties.debugOutput) {
            if(m_filterProperties.debugOutput) {
                if(m_filterProperties.debugOutput) {
                    LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormatTwo.nWidth, m_sOutputFormatTwo.nHeight, m_sOutputFormatTwo.nBytesPerLine, m_sOutputFormatTwo.nSize, m_sOutputFormatTwo.nPixelFormat));
                }
            }
        }
        //set output format for output pin
        m_oVideoOutputPinTwo.SetFormat(&m_sOutputFormatTwo, NULL);
    }
    RETURN_NOERROR;
}

tResult StopLineDetector::TransmitStopline(const tBool StopLine, tTimeStamp timestamp)
{
    __synchronized_obj(critBoolOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputBoolSet)
        {
            pCoderOutput->GetID("bValue",m_szIDOutputBoolValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDOutputBoolTimeStamp);
            m_szIDsOutputBoolSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputBoolValue, (tVoid*)&StopLine);
        pCoderOutput->Set(m_szIDOutputBoolTimeStamp, (tVoid*)&timestamp);


        pMediaSample->SetTime(timestamp);
    }
    RETURN_IF_FAILED(m_oBoolValuePin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult StopLineDetector::TransmitOvertakingPossible(const tBool overtakingIsPossible)
{
    __synchronized_obj(critOvertakingPossibleOut);

    PRINT1("Transmit: %d", overtakingIsPossible);
    

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        if (!m_OvertakingPossibleOutSet)
        {
            pCoderOutput->GetID("bValue",m_OvertakingPossibleOutBuffer);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_OvertakingPossibleOutBufferTs);
            m_OvertakingPossibleOutSet = tTrue;
        }
        pCoderOutput->Set(m_OvertakingPossibleOutBuffer, (tVoid*)&overtakingIsPossible);
        
        tTimeStamp ts = GetTime();
        pCoderOutput->Set(m_OvertakingPossibleOutBufferTs, (tVoid*)&ts);
        pMediaSample->SetTime(ts);
    }
    RETURN_IF_FAILED(m_OvertakingPossibleOutPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult StopLineDetector::TransmitStopDistance(tFloat32 distance)
{
    tTimeStamp ts = GetTime();
    __synchronized_obj(critDistanceOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderOutput);

        if (!m_DistanceOutSet)
        {
            pCoderOutput->GetID("f32Value",m_DistanceOutBuffer);
            pCoderOutput->GetID("ui32ArduinoTimestamp",m_DistanceOutBufferTs);
            m_DistanceOutSet = tTrue;
        }
        pCoderOutput->Set(m_DistanceOutBuffer, (tVoid*)&distance);
        pCoderOutput->Set(m_DistanceOutBufferTs, (tVoid*)&ts);


        pMediaSample->SetTime(GetTime());
    }
    RETURN_IF_FAILED(m_DistanceOut.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tUInt32 StopLineDetector::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 StopLineDetector::Betrag(tFloat32 input)
{
    tFloat32 Betrag = (input >= 0) ? input : -input;
    return Betrag;
}
