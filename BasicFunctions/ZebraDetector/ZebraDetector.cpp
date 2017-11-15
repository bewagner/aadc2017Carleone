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
#include "ZebraDetector.h"
#include <iostream>

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("ZebraDetector","ZebraDetector",ZebraDetector)



ZebraDetector::ZebraDetector(const tChar* __info) : cFilter(__info)
{
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
	SetPropertyStr("ROI::StraightnessThreshold" NSSUBPROP_DESCRIPTION, "Threshold of ZebraDetection Straightness");
	SetPropertyBool("ROI::StraightnessThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("ROI::DynamicMultiplier", 10);
	SetPropertyStr("ROI::DynamicMultiplier" NSSUBPROP_DESCRIPTION, "DynamicMultiplier");
	SetPropertyBool("ROI::DynamicMultiplier" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::MaxDynamicWidth", 100);
	SetPropertyStr("ROI::MaxDynamicWidth" NSSUBPROP_DESCRIPTION, "MaxDynamicWidth");
	SetPropertyBool("ROI::MaxDynamicWidth" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROIONE::PointOneX", 580);
	SetPropertyStr("ROIONE::PointOneX" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROIONE::PointOneX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROIONE::PointTwoX", 610);
	SetPropertyStr("ROIONE::PointTwoX" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROIONE::PointTwoX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROIONE::PointOneY", 520);
	SetPropertyStr("ROIONE::PointOneY" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROIONE::PointOneY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROIONE::PointTwoY", 550);
	SetPropertyStr("ROIONE::PointTwoY" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROIONE::PointTwoY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointOneX", 610);
	SetPropertyStr("ROITWO::PointOneX" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROITWO::PointOneX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointTwoX", 635);
	SetPropertyStr("ROITWO::PointTwoX" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROITWO::PointTwoX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointOneY", 520);
	SetPropertyStr("ROITWO::PointOneY" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROITWO::PointOneY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITWO::PointTwoY", 550);
	SetPropertyStr("ROITWO::PointTwoY" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROITWO::PointTwoY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITHREE::PointOneX", 635);
	SetPropertyStr("ROITHREE::PointOneX" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROITHREE::PointOneX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITHREE::PointTwoX", 660);
	SetPropertyStr("ROITHREE::PointTwoX" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROITHREE::PointTwoX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITHREE::PointOneY", 520);
	SetPropertyStr("ROITHREE::PointOneY" NSSUBPROP_DESCRIPTION, "P1");
	SetPropertyBool("ROITHREE::PointOneY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROITHREE::PointTwoY", 550);
	SetPropertyStr("ROITHREE::PointTwoY" NSSUBPROP_DESCRIPTION, "P2");
	SetPropertyBool("ROITHREE::PointTwoY" NSSUBPROP_ISCHANGEABLE, tTrue);


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

	SetPropertyInt("Zebra::NumberOfEdges", 5);
	SetPropertyStr("Zebra::NumberOfEdges" NSSUBPROP_DESCRIPTION, "NumberOfEdges");
	SetPropertyBool("Zebra::NumberOfEdges" NSSUBPROP_ISCHANGEABLE, tTrue);



	HorizontalLineDetected = tFalse;
	m_DistanceOutSet = tFalse;
	m_szIDsOutputBoolSet = tFalse;
	m_ZebraSignBufferSet = tFalse;
	Offset = 0;

}

ZebraDetector::~ZebraDetector()
{
}

tResult ZebraDetector::Start(__exception)
{

	return cFilter::Start(__exception_ptr);
}

tResult ZebraDetector::Stop(__exception)
{
	//destroyWindow("Debug");
	return cFilter::Stop(__exception_ptr);
}
tResult ZebraDetector::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{

		RETURN_IF_FAILED(m_oVideoInputPinTwo.Create("Video_Input_Two", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPinTwo));

		// Video Output
		RETURN_IF_FAILED(m_oVideoOutputPinTwo.Create("Video_Output_Two_Debug", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPinTwo));

		// GCL Output
		m_oGCLOutputPin.Create("GCL", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this));
		RegisterPin(&m_oGCLOutputPin);

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

		//get mediatype description
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
		RETURN_IF_FAILED(pTypeTSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

		//create pin for output
		RETURN_IF_FAILED(m_oBoolValuePin.Create("BoolValue",pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oBoolValuePin));
	}

	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageGraphReady)
	{
		// get the image format of the input video pin
		cObjectPtr<IMediaType> pTypeTwo;
		RETURN_IF_FAILED(m_oVideoInputPinTwo.GetMediaType(&pTypeTwo));

		cObjectPtr<IMediaTypeVideo> pTypeVideoTwo;

		RETURN_IF_FAILED(pTypeTwo->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideoTwo));

		// set the image format of the input video pin
		if (IS_FAILED(UpdateInputImageFormatTwo(pTypeVideoTwo->GetFormat())))
		{
			LOG_ERROR("Invalid Input Format for this filter");
		}
	}

	RETURN_NOERROR;
}



tResult ZebraDetector::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	if (eStage == StageGraphReady)
	{
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult ZebraDetector::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	tTimeStamp inputTimestamp = pMediaSample->GetTime();

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pSource == &m_oVideoInputPinTwo)
		{
			if (m_sInputFormatTwo.nPixelFormat == IImage::PF_UNKNOWN)
			{
				RETURN_IF_FAILED(UpdateInputImageFormatTwo(m_oVideoInputPinTwo.GetFormat()));
			}
			ProcessVideoTwo(pMediaSample, inputTimestamp);
		}
	}
	else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		if (pSource == &m_oVideoInputPinTwo)
		{
			//the input format was changed, so the imageformat has to changed in this filter also
			RETURN_IF_FAILED(UpdateInputImageFormatTwo(m_oVideoInputPinTwo.GetFormat()));
		}
		ProcessVideoTwo(pMediaSample, inputTimestamp);
	}

	RETURN_NOERROR;
}

tResult ZebraDetector::PropertyChanged(const tChar* strName)
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
	else if (cString::IsEqual(strName, "Corners::MaxDistance"))
		m_filterProperties.distanceOfCorners = GetPropertyFloat("Corners::MaxDistance");
	else if (cString::IsEqual(strName, "Corners::NumberOfCorners"))
		m_filterProperties.numberOfCorners = GetPropertyInt("Corners::NumberOfCorners");
	else if (cString::IsEqual(strName, "ROI::DynamicMultiplier"))
		m_filterProperties.ROIDynamicMultiplier = GetPropertyInt("ROI::DynamicMultiplier");
	else if (cString::IsEqual(strName, "ROI::MaxDynamicWidth"))
		m_filterProperties.ROIMaxOffset = GetPropertyInt("ROI::MaxDynamicWidth");
	else if (cString::IsEqual(strName, "ROIONE::PointOneX"))
		m_filterProperties.ROIOneP1X = GetPropertyInt("ROIONE::PointOneX");
	else if (cString::IsEqual(strName, "ROIONE::PointTwoX"))
		m_filterProperties.ROIOneP2X = GetPropertyInt("ROIONE::PointTwoX");
	else if (cString::IsEqual(strName, "ROIONE::PointOneY"))
		m_filterProperties.ROIOneP1Y = GetPropertyInt("ROIONE::PointOneY");
	else if (cString::IsEqual(strName, "ROIONE::PointTwoY"))
		m_filterProperties.ROIOneP2Y = GetPropertyInt("ROIONE::PointTwoY");
	else if (cString::IsEqual(strName, "ROITWO::PointOneX"))
		m_filterProperties.ROITwoP1X = GetPropertyInt("ROITWO::PointOneX");
	else if (cString::IsEqual(strName, "ROITWO::PointTwoX"))
		m_filterProperties.ROITwoP2X = GetPropertyInt("ROITWO::PointTwoX");
	else if (cString::IsEqual(strName, "ROITWO::PointOneY"))
		m_filterProperties.ROITwoP1Y = GetPropertyInt("ROITWO::PointOneY");
	else if (cString::IsEqual(strName, "ROITWO::PointTwoY"))
		m_filterProperties.ROITwoP2Y = GetPropertyInt("ROITWO::PointTwoY");
	else if (cString::IsEqual(strName, "ROITHREE::PointOneX"))
		m_filterProperties.ROIThreeP1X = GetPropertyInt("ROITHREE::PointOneX");
	else if (cString::IsEqual(strName, "ROITHREE::PointTwoX"))
		m_filterProperties.ROIThreeP2X = GetPropertyInt("ROITHREE::PointTwoX");
	else if (cString::IsEqual(strName, "ROITHREE::PointOneY"))
		m_filterProperties.ROIThreeP1Y = GetPropertyInt("ROITHREE::PointOneY");
	else if (cString::IsEqual(strName, "ROITHREE::PointTwoY"))
		m_filterProperties.ROIThreeP2Y = GetPropertyInt("ROITHREE::PointTwoY");
	else if (cString::IsEqual(strName, "Zebra::NumberOfEdges"))
		m_filterProperties.numberOfEdges = GetPropertyInt("Zebra::NumberOfEdges");


	RETURN_NOERROR;
}

tResult ZebraDetector::ProcessVideoTwo(IMediaSample* pSample, tTimeStamp inputTimestamp)
{
	RETURN_IF_POINTER_NULL(pSample);
	// new image for result
	cv::Mat GreyImage;
	cv::Mat BinaryImage;
	const tVoid* l_pSrcBuffer;
	cv::Mat outputImage1;
	cv::Mat outputImage2;
	cv::Mat outputImage3;
	cv::Mat outputImage4;


	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{
		//convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImageTwo.total() * m_inputImageTwo.elemSize()) == m_sInputFormatTwo.nSize)
		{
			m_inputImageTwo.data = (uchar*)(l_pSrcBuffer);
		}
		cvtColor(m_inputImageTwo, GreyImage, CV_RGB2GRAY);
		int width1 = abs(m_filterProperties.ROIOneP1X - m_filterProperties.ROIOneP2X);
		int height1 = abs(m_filterProperties.ROIOneP1Y - m_filterProperties.ROIOneP2Y);

		int width2 = abs(m_filterProperties.ROITwoP1X - m_filterProperties.ROITwoP2X);
		int height2 = abs(m_filterProperties.ROITwoP1Y - m_filterProperties.ROITwoP2Y);

		int width3 = abs(m_filterProperties.ROIThreeP1X - m_filterProperties.ROIThreeP2X);
		int height3 = abs(m_filterProperties.ROIThreeP1Y - m_filterProperties.ROIThreeP2Y);

		cv::Rect Roi1 = Rect(m_filterProperties.ROIOneP1X,m_filterProperties.ROIOneP1Y,width1,height1);
		cv::Rect Roi2 = Rect(m_filterProperties.ROITwoP1X,m_filterProperties.ROITwoP1Y,width2,height2);
		cv::Rect Roi3 = Rect(m_filterProperties.ROIThreeP1X,m_filterProperties.ROIThreeP1Y,width3,height3);
		//	cv::threshold(GreyImage,GreyImage,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
		//cv::Mat croppedBinary1(GreyImage, Roi1);
		cv::Mat croppedBinary2(GreyImage, Roi2);
		cv::Mat croppedBinary3(GreyImage, Roi3);
		//cv::threshold(croppedBinary1,croppedBinary1,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
		cv::threshold(croppedBinary2,croppedBinary2,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);

		tInt counter = 0;
		tBool white = tFalse;
		int heightTwoHalf = floor((float)height2/2.0);
		for(unsigned int j =0; j < (abs(m_filterProperties.ROITwoP1X - m_filterProperties.ROITwoP2X)); ++j)
		{
			int nmbr_of_whites = (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf-3,j))[0] >= 80) + (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf-2,j))[0] >= 80) + (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf-1,j))[0] >= 80) + (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf,j))[0]>= 80) + (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf+1,j))[0]>= 80)+ (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf+2,j))[0] >= 80) + (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf+3,j))[0] >= 80);
			//int nmbr_of_whites = (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf-1,j))[0] >= 80) + (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf,j))[0]>= 80) + (int) (cv::mean(croppedBinary2.at<unsigned char>(heightTwoHalf+1,j))[0]>= 80);
			int nmbr_of_blacks = 7 - nmbr_of_whites;

			if (!white && nmbr_of_blacks >5)
			{
				continue;
			}
			else if(!white && nmbr_of_whites > 5)
			{
				white = tTrue;
				counter ++;
			} else if (white && nmbr_of_whites > 5)
			{
				continue;
			} else if(white && nmbr_of_blacks > 5)
			{
				white = tFalse;
			}
		}

		//cv::threshold(croppedBinary3,croppedBinary3,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
		//cv::Scalar l_mean = cv::mean(croppedBinary1);
		//cv::Scalar m_mean = cv::mean(croppedBinary2);
		//cv::Scalar r_mean = cv::mean(croppedBinary3);
		//PRINT1("%i ",counter);
		//croppedBinary1.copyTo(outputImage1);
		croppedBinary2.copyTo(outputImage2);
		//croppedBinary3.copyTo(outputImage3);
		//GreyImage.copyTo(outputImage4);
		if(counter >=5 && counter <=7)
		{
			PRINT1("counter: %i", counter);		
			TransmitZebra(tTrue);
		}
	}
	pSample->Unlock(l_pSrcBuffer);

	if (!outputImage2.empty() && m_oVideoOutputPinTwo.IsConnected())
	{
		UpdateOutputImageFormatTwo(outputImage2);

		//create a cImage from CV Matrix
		cImage newImage;
		newImage.Create(m_sOutputFormatTwo.nWidth, m_sOutputFormatTwo.nHeight, m_sOutputFormatTwo.nBitsPerPixel, m_sOutputFormatTwo.nBytesPerLine, outputImage2.data);

		//create the new media sample
		cObjectPtr<IMediaSample> pMediaSample;
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
		//updating media sample
		RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
		//transmitting
		RETURN_IF_FAILED(m_oVideoOutputPinTwo.Transmit(pMediaSample));

		outputImage2.release();
	}
	RETURN_NOERROR;


}

tResult ZebraDetector::findLinePoints(const vector<tInt>& detectionLines, const cv::Mat& image, vector<cPoint>& detectedLinePoints, tTimeStamp inputTimestamp)
{
	//tFloat32 ZebraSignAngle = 0;
	//iterate through the calculated horizontal lines
	for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
	{

		uchar ucLastVal = 0;
		//transpose(image,image);
		// create vector with line data
		const uchar* p = image.ptr<uchar>(*nline, m_filterProperties.ROIOffsetY);
		//const uchar* q = image.ptr<uchar>(m_filterProperties.ROIOffsetY +m_filterProperties.ROIHeight, *nline);
		std::vector<uchar> lineData(p,p+m_filterProperties.ROIHeight);
		//transpose(image,image);

		tBool detectedStartCornerLine = tFalse;
		tInt columnStartCornerLine = 0;

		for (std::vector<uchar>::iterator lineIterator = lineData.begin(); lineIterator != lineData.end(); lineIterator++)
		{
			uchar ucCurrentVal = *lineIterator;
			tInt currentIndex = tInt(std::distance(lineData.begin(), lineIterator));
			//look for transition from dark to bright -> start of line corner
			if ((ucCurrentVal - ucLastVal) > m_filterProperties.minLineContrast)
			{
				detectedStartCornerLine = tTrue;
				columnStartCornerLine = currentIndex;
			}//look for transition from bright to dark -> end of line
			else if ((ucLastVal - ucCurrentVal) > m_filterProperties.minLineContrast && detectedStartCornerLine)
			{
				//we already have the start corner of line, so check the width of detected line
				if ((abs(columnStartCornerLine - currentIndex) > m_filterProperties.minLineWidth)
						&& (abs(columnStartCornerLine - currentIndex) < m_filterProperties.maxLineWidth))
				{
					detectedLinePoints.push_back(cPoint(*nline,tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
							m_filterProperties.ROIOffsetY)));

					detectedStartCornerLine = tFalse;
					columnStartCornerLine = 0;


				}
			}
			//we reached maximum line width limit, stop looking for end of line
			if (detectedStartCornerLine &&
					abs(columnStartCornerLine - currentIndex) > m_filterProperties.maxLineWidth)
			{
				detectedStartCornerLine = tFalse;
				columnStartCornerLine = 0;
			}
			ucLastVal = ucCurrentVal;


		}

	}


	RETURN_NOERROR;
}



tResult ZebraDetector::getDetectionLines(vector<tInt>& detectionLines)
{
	tInt distanceBetweenDetectionLines = m_filterProperties.ROIWidth / (m_filterProperties.detectionLines + 1);

	for (uint i = 1; i <= m_filterProperties.detectionLines; i++)
	{
		detectionLines.push_back(m_filterProperties.ROIOffsetX + i * distanceBetweenDetectionLines);
	}
	RETURN_NOERROR;
}


tResult ZebraDetector::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		//update member variable
		m_sInputFormat = (*pFormat);
		if(m_filterProperties.debugOutput) {
			if(m_filterProperties.debugOutput) {
				LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
			}
		}
		//create the input matrix
		RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
	}
	RETURN_NOERROR;
}
tResult ZebraDetector::UpdateInputImageFormatTwo(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		//update member variable
		m_sInputFormatTwo = (*pFormat);
		if(m_filterProperties.debugOutput) {
			if(m_filterProperties.debugOutput) {
				LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormatTwo.nWidth, m_sInputFormatTwo.nHeight, m_sInputFormatTwo.nBytesPerLine, m_sInputFormatTwo.nSize, m_sInputFormatTwo.nPixelFormat));
			}
		}
		//create the input matrix
		RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormatTwo, m_inputImageTwo));
	}
	RETURN_NOERROR;
}
tResult ZebraDetector::transmitGCL(const vector<tInt>& detectionLines, const vector<Point2f>& detectedLinePoints)
{

	IDynamicMemoryBlock* pGCLCmdDebugInfo;

	cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);

	//set color
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Red.GetRGBA());
	//show roi
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetX + Offset, m_filterProperties.ROIOffsetY,
			m_filterProperties.ROIOffsetX + m_filterProperties.ROIWidth + Offset, m_filterProperties.ROIOffsetY + m_filterProperties.ROIHeight);

	//show detection lines
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Yellow.GetRGBA());
	for (vector<tInt>::const_iterator it = detectionLines.begin(); it != detectionLines.end(); it++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE,*it, m_filterProperties.ROIOffsetY,*it , m_filterProperties.ROIOffsetY + m_filterProperties.ROIHeight);
	}

	/* //show detected lane points
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Blue.GetRGBA());
	for (vector<Point2f>::const_iterator LinePointsIter = detectedLinePoints.begin(); LinePointsIter != detectedLinePoints.end(); LinePointsIter++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE,LinePointsIter->x, LinePointsIter->y, 20);
		//cout << "gcl output!!!!!elf" << endl;
	}
	 */
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Blue.GetRGBA());
	//show minimum and maximum line width
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetX, m_filterProperties.ROIOffsetY,
			m_filterProperties.ROIOffsetX + m_filterProperties.maxLineWidth, m_filterProperties.ROIOffsetY - 20);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetX + m_filterProperties.ROIWidth - m_filterProperties.maxLineWidth, m_filterProperties.ROIOffsetY,
			m_filterProperties.ROIOffsetX + m_filterProperties.ROIWidth, m_filterProperties.ROIOffsetY - 20);


	//show minimum and maximum line width at left and right roi
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetX + m_filterProperties.ROIWidth - m_filterProperties.minLineWidth, m_filterProperties.ROIOffsetY - 20,
			m_filterProperties.ROIOffsetX + m_filterProperties.ROIWidth, m_filterProperties.ROIOffsetY - 40);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetX, m_filterProperties.ROIOffsetY - 20,
			m_filterProperties.ROIOffsetX + m_filterProperties.minLineWidth, m_filterProperties.ROIOffsetY - 40);


	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);

	//alloc media sample and transmit it over output pin
	cObjectPtr<IMediaSample> pSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));
	RETURN_IF_FAILED(pSample->Update(_clock->GetStreamTime(),
			pGCLCmdDebugInfo->GetPtr(), (tInt)pGCLCmdDebugInfo->GetSize(), IMediaSample::MSF_None));
	RETURN_IF_FAILED(m_oGCLOutputPin.Transmit(pSample));

	cGCLWriter::FreeDynamicMemoryBlock(pGCLCmdDebugInfo);

	RETURN_NOERROR;
}

tResult ZebraDetector::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
	//check if pixelformat or size has changed
	if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
	{
		Mat2BmpFormat(outputImage, m_sOutputFormat);

		if(m_filterProperties.debugOutput) {
			if(m_filterProperties.debugOutput) {
				LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
			}
		}
		//set output format for output pin
		m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
	}
	RETURN_NOERROR;
}

tResult ZebraDetector::UpdateOutputImageFormatTwo(const cv::Mat& outputImage)
{
	//check if pixelformat or size has changed
	if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormatTwo.nSize)
	{
		Mat2BmpFormat(outputImage, m_sOutputFormatTwo);

		if(m_filterProperties.debugOutput) {
			if(m_filterProperties.debugOutput) {
				LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormatTwo.nWidth, m_sOutputFormatTwo.nHeight, m_sOutputFormatTwo.nBytesPerLine, m_sOutputFormatTwo.nSize, m_sOutputFormatTwo.nPixelFormat));
			}
		}
		//set output format for output pin
		m_oVideoOutputPinTwo.SetFormat(&m_sOutputFormatTwo, NULL);
	}
	RETURN_NOERROR;
}

tResult ZebraDetector::UpdateOutputImageFormatThree(const cv::Mat& outputImage)
{
	//check if pixelformat or size has changed
	if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
	{
		Mat2BmpFormat(outputImage, m_sOutputFormatThree);

		if(m_filterProperties.debugOutput) {
			if(m_filterProperties.debugOutput) {
				LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormatThree.nWidth, m_sOutputFormatThree.nHeight, m_sOutputFormatThree.nBytesPerLine, m_sOutputFormatThree.nSize, m_sOutputFormatThree.nPixelFormat));
			}
		}
		//set output format for output pin
		m_oVideoOutputPinThree.SetFormat(&m_sOutputFormatThree, NULL);
	}
	RETURN_NOERROR;
}

tResult ZebraDetector::UpdateOutputImageFormatFour(const cv::Mat& outputImage)
{
	//check if pixelformat or size has changed
	if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
	{
		Mat2BmpFormat(outputImage, m_sOutputFormatFour);

		if(m_filterProperties.debugOutput) {
			if(m_filterProperties.debugOutput) {
				LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormatFour.nWidth, m_sOutputFormatFour.nHeight, m_sOutputFormatFour.nBytesPerLine, m_sOutputFormatFour.nSize, m_sOutputFormatFour.nPixelFormat));
			}
		}
		//set output format for output pin
		m_oVideoOutputPinFour.SetFormat(&m_sOutputFormatFour, NULL);
	}
	RETURN_NOERROR;
}

tResult ZebraDetector::TransmitZebra(const tBool Zebra)
{
	__synchronized_obj(CritZebraOut);
	cObjectPtr<IMediaSample> pMediaSample;
	tTimeStamp ZebraTime = GetTime();
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
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_timestampBuffer);
			m_szIDsOutputBoolSet = tTrue;
		}
		pCoderOutput->Set(m_szIDOutputBoolValue, (tVoid*)&Zebra);
		pCoderOutput->Set(m_timestampBuffer, (tVoid*)&ZebraTime);
	}

	RETURN_IF_FAILED(m_oBoolValuePin.Transmit(pMediaSample));
	RETURN_NOERROR;
}

tUInt32 ZebraDetector::GetTime()
{
	return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 ZebraDetector::Betrag(tFloat32 input)
{
	tFloat32 Betrag = (input >= 0) ? input : -input;
	return Betrag;
}
