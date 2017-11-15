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
#include "CrossroadDetector.h"
#include <iostream>

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("CrossroadDetector","CrossroadDetector",CrossroadDetector)



CrossroadDetector::CrossroadDetector(const tChar* __info) : cFilter(__info)
{
	SetPropertyBool("DEBUG::DebutOutput", tFalse); 
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?"); 
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("DEBUG::DebutOutput", tFalse); 
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?"); 
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	SendLock = tFalse;

	CrossPointHL = tFalse;
	CrossPointVL = tFalse;
	CrossPointHR = tFalse;
	CrossPointVR = tFalse;

	// X/Y Offset for left line horizontal
	SetPropertyInt("ROI::HLXOffset", 150);
	SetPropertyStr("ROI::HLXOffset" NSSUBPROP_DESCRIPTION, "Left - X Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::HLXOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("ROI::HLYOffset", 575);
	SetPropertyStr("ROI::HLYOffset" NSSUBPROP_DESCRIPTION, "Left - Y Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::HLYOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	// X/Y Offset for left line vertikal
	SetPropertyInt("ROI::VLXOffset", 300);
	SetPropertyStr("ROI::VLXOffset" NSSUBPROP_DESCRIPTION, "Right - X Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::VLXOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("ROI::VLYOffset", 575);
	SetPropertyStr("ROI::VLYOffset" NSSUBPROP_DESCRIPTION, "Right - Y Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::VLYOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	// X/Y Offset for right line horizontal
	SetPropertyInt("ROI::HRXOffset", 700);
	SetPropertyStr("ROI::HRXOffset" NSSUBPROP_DESCRIPTION, "Left - X Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::HRXOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("ROI::HRYOffset", 575);
	SetPropertyStr("ROI::HRYOffset" NSSUBPROP_DESCRIPTION, "Left - Y Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::HRYOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	// X/Y Offset for right line vertikal
	SetPropertyInt("ROI::VRXOffset", 700);
	SetPropertyStr("ROI::VRXOffset" NSSUBPROP_DESCRIPTION, "Right - X Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::VRXOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("ROI::VRYOffset", 575);
	SetPropertyStr("ROI::VRYOffset" NSSUBPROP_DESCRIPTION, "Right - Y Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::VRYOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::WidthH", 200);
	SetPropertyStr("ROI::WidthH" NSSUBPROP_DESCRIPTION, "Width of the Region of Interest Rectangular");
	SetPropertyBool("ROI::WidthH" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("ROI::WidthV", 50);
	SetPropertyStr("ROI::WidthV" NSSUBPROP_DESCRIPTION, "Width of the Region of Interest Rectangular");
	SetPropertyBool("ROI::WidthV" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::HeightH", 50);
	SetPropertyStr("ROI::HeightH" NSSUBPROP_DESCRIPTION, "Height of the Region of Interest Rectangular");
	SetPropertyBool("ROI::HeightH" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("ROI::HeightV", 200);
	SetPropertyStr("ROI::HeightV" NSSUBPROP_DESCRIPTION, "Height of the Region of Interest Rectangular");
	SetPropertyBool("ROI::HeightV" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("ROI::StraightnessThreshold", 30);
	SetPropertyStr("ROI::StraightnessThreshold" NSSUBPROP_DESCRIPTION, "Threshold of CrossroadDetection Straightness");
	SetPropertyBool("ROI::StraightnessThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyInt("Algorithm::Detection Lines", 30);
	SetPropertyStr("Algorithm::Detection Lines" NSSUBPROP_DESCRIPTION, "number of detection lines searched in ROI");
	SetPropertyBool("Algorithm::Detection Lines" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Detection Lines" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Minimum Line Width", 1);
	SetPropertyStr("Algorithm::Minimum Line Width" NSSUBPROP_DESCRIPTION, "Minimum Line Width in Pixel");
	SetPropertyBool("Algorithm::Minimum Line Width" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Minimum Line Width" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Maximum Line Width", 30);
	SetPropertyStr("Algorithm::Maximum Line Width" NSSUBPROP_DESCRIPTION, "Maximum Line Width in Pixel");
	SetPropertyBool("Algorithm::Maximum Line Width" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Maximum Line Width" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Minimum Line Contrast", 30);
	SetPropertyStr("Algorithm::Minimum Line Contrast" NSSUBPROP_DESCRIPTION, "Mimimum line contrast in gray Values");
	SetPropertyBool("Algorithm::Minimum Line Contrast" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MIN, 1);
	SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MAX, 255);

	SetPropertyInt("Algorithm::Image Binarization Threshold", 180);
	SetPropertyStr("Algorithm::Image Binarization Threshold" NSSUBPROP_DESCRIPTION, "Threshold for image binarization");
	SetPropertyBool("Algorithm::Image Binarization Threshold" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MIN, 1);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MAX, 255);


	SetPropertyInt("Canny::Threshold", 100);
	SetPropertyStr("Canny::Threshold" NSSUBPROP_DESCRIPTION, "Threshold of Edge Detection");
	SetPropertyBool("Canny::Threshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Canny::ThresholdRatio", 3);
	SetPropertyStr("Canny::ThresholdRatio" NSSUBPROP_DESCRIPTION, "Ratio of Edge Detection Threshold");
	SetPropertyBool("Canny::ThresholdRatio" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Canny::KernelSize", 3);
	SetPropertyStr("Canny::KernelSize" NSSUBPROP_DESCRIPTION, "KernelSize of Edge Detection");
	SetPropertyBool("Canny::KernelSize" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Hough::AccumulatorThreshold", 100);
	SetPropertyStr("Hough::AccumulatorThreshold" NSSUBPROP_DESCRIPTION, "Amount of Intersections needed to detect a line");
	SetPropertyBool("Hough::AccumulatorThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);
}

CrossroadDetector::~CrossroadDetector()
{
}

tResult CrossroadDetector::Start(__exception)
{

	return cFilter::Start(__exception_ptr);
}

tResult CrossroadDetector::Stop(__exception)
{
	//destroyWindow("Debug");
	return cFilter::Stop(__exception_ptr);
}
tResult CrossroadDetector::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

		// Video Output
		RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output_Debug", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

		// GCL Output
		m_oGCLOutputPin.Create("GCL", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this));
		RegisterPin(&m_oGCLOutputPin);

		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
		//get description
		tChar const * strDescignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");

		// checks if exists
		RETURN_IF_POINTER_NULL(strDescignalValue);

		//get mediatype
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		//get mediatype description
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));

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
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		// set the image format of the input video pin
		if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
		{
			LOG_ERROR("Invalid Input Format for this filter");
		}
	}

	RETURN_NOERROR;
}



tResult CrossroadDetector::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	if (eStage == StageGraphReady)
	{
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult CrossroadDetector::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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

			ProcessVideo(pMediaSample, inputTimestamp);
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

tResult CrossroadDetector::PropertyChanged(const tChar* strName)
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
	if (cString::IsEqual(strName, "ROI::WidthH"))
		m_filterProperties.ROIWidthH = GetPropertyInt("ROI::WidthH");
	else if (cString::IsEqual(strName, "ROI::WidthV"))
		m_filterProperties.ROIWidthV = GetPropertyInt("ROI::WidthV");
	else if (cString::IsEqual(strName, "ROI::HeightH"))
		m_filterProperties.ROIHeightH = GetPropertyInt("ROI::HeightH");
	else if (cString::IsEqual(strName, "ROI::HeightV"))
		m_filterProperties.ROIHeightV = GetPropertyInt("ROI::HeightV");

	else if (cString::IsEqual(strName, "ROI::HLXOffset"))
		m_filterProperties.ROIOffsetXHL = GetPropertyInt("ROI::HLXOffset");
	else if (cString::IsEqual(strName, "ROI::HLYOffset"))
		m_filterProperties.ROIOffsetYHL = GetPropertyInt("ROI::HLYOffset");
	else if (cString::IsEqual(strName, "ROI::VLXOffset"))
		m_filterProperties.ROIOffsetXVL = GetPropertyInt("ROI::VLXOffset");
	else if (cString::IsEqual(strName, "ROI::VLYOffset"))
		m_filterProperties.ROIOffsetYVL = GetPropertyInt("ROI::VLYOffset");
	else if (cString::IsEqual(strName, "ROI::HRXOffset"))
		m_filterProperties.ROIOffsetXHR = GetPropertyInt("ROI::HRXOffset");
	else if (cString::IsEqual(strName, "ROI::HRYOffset"))
		m_filterProperties.ROIOffsetYHR = GetPropertyInt("ROI::HRYOffset");
	else if (cString::IsEqual(strName, "ROI::VRXOffset"))
		m_filterProperties.ROIOffsetXVR = GetPropertyInt("ROI::VRXOffset");
	else if (cString::IsEqual(strName, "ROI::VRYOffset"))
		m_filterProperties.ROIOffsetYVR = GetPropertyInt("ROI::VRYOffset");
	else if (cString::IsEqual(strName, "ROI::StraightnessThreshold"))
		m_filterProperties.StraightnessThreshold = GetPropertyInt("ROI::StraightnessThreshold");

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
		m_filterProperties.Threshold = GetPropertyInt("Canny::Threshold");
	else if (cString::IsEqual(strName, "Canny::ThresholdRatio"))
		m_filterProperties.ThresholdRatio = GetPropertyInt("Canny::ThresholdRatio");
	else if (cString::IsEqual(strName, "Canny::KernelSize"))
		m_filterProperties.KernelSize = GetPropertyInt("Canny::KernelSize");

	else if (cString::IsEqual(strName, "Hough::AccumulatorThreshold"))
		m_filterProperties.AccumulatorThreshold = GetPropertyInt("Hough::AccumulatorThreshold");

	RETURN_NOERROR;
}


tResult CrossroadDetector::ProcessVideo(IMediaSample* pSample, tTimeStamp inputTimestamp)
{
	RETURN_IF_POINTER_NULL(pSample);
	// new image for result
	cv::Mat outputImage;

	// here we store the pixel lines in the image where we search for lanes
	vector<tInt> detectionLinesHL;
	vector<tInt> detectionLinesVL;
	vector<tInt> detectionLinesHR;
	vector<tInt> detectionLinesVR;

	// here we have all the detected line points
	vector<cPoint> detectedLinePointsHL;
	vector<cPoint> detectedLinePointsVL;
	vector<cPoint> detectedLinePointsHR;
	vector<cPoint> detectedLinePointsVR;
	vector<cPoint> detectedLinePoints;


	const tVoid* l_pSrcBuffer;

	//receiving data from input sample, and saving to TheInputImage
	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{
		//convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
		{
			m_inputImage.data = (uchar*)(l_pSrcBuffer);

			/*
			  GaussianBlur(m_inputImage, m_inputImage,size(7,7),1.5,1.5);
			  Canny(m_inputImage, outputImage, m_filterProperties.Threshold, m_filterProperties.ThresholdRatio, m_filterProperties.KernelSize);
			  vector<Vec2f> lines;
			  HoughLinesP(outputImage, lines, 1, CV_PI/180, m_filterProperties.AccumulatorThreshold, 50, 50 );
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
			  line( outputImage, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
			  } */
			//outputImage = m_inputImage;
			//convert to grayscale
			cvtColor(m_inputImage, outputImage, CV_RGB2GRAY);
			getDetectionLinesVL(detectionLinesVL);
			getDetectionLinesVR(detectionLinesVR);
			findLinePointsVR(detectionLinesVR, outputImage, detectedLinePointsVR, inputTimestamp);
			findLinePointsVL(detectionLinesVL, outputImage, detectedLinePointsVL, inputTimestamp);
			transpose(outputImage,outputImage);
			getDetectionLinesHL(detectionLinesHL);
			getDetectionLinesHR(detectionLinesHR);
			findLinePointsHL(detectionLinesHL, outputImage, detectedLinePointsHL, inputTimestamp);
			findLinePointsHR(detectionLinesHR, outputImage, detectedLinePointsHR, inputTimestamp);
			transpose(outputImage,outputImage);

			///
			tFloat32 straightness = 0;
			tFloat32 average = 0;
			if (detectedLinePointsHL.size() > 2*m_filterProperties.detectionLines / 3 && !SendLock)
			{
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsHL.begin(); Iter != detectedLinePointsHL.end(); Iter++)
				{
					average += Iter->GetY();
				}
				average /= detectedLinePointsHL.size();
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsHL.begin(); Iter != detectedLinePointsHL.end(); Iter++)
				{
					straightness += (average - Iter->GetY())*(average - Iter->GetY());
				}
				if (straightness <= m_filterProperties.StraightnessThreshold )
				{
					CrossPointHL = tTrue;
					// cout << "CrossPoint HL gefunden" << endl;
					SendLock = tTrue;
				}
				else
				{
					detectedLinePointsHL.clear();
				}
				straightness = 0;
			}
			else
			{
				detectedLinePointsHL.clear();
				SendLock = tFalse;
			}
			//TransmitStopline(tFalse, inputTimestamp);

			///
			straightness = 0;
			average = 0;
			if (detectedLinePointsVL.size() > 2*m_filterProperties.detectionLines / 3 && !SendLock)
			{
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsVL.begin(); Iter != detectedLinePointsVL.end(); Iter++)
				{
					average += Iter->GetY();
				}
				average /= detectedLinePointsVL.size();
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsVL.begin(); Iter != detectedLinePointsVL.end(); Iter++)
				{
					straightness += (average - Iter->GetY())*(average - Iter->GetY());
				}
				if (straightness <= m_filterProperties.StraightnessThreshold )
				{
					CrossPointVL = tTrue;
					// cout << "CrossPoint VL gefunden" << endl;
					SendLock = tTrue;
				}
				else
				{
					detectedLinePointsVL.clear();
				}
				straightness = 0;
			}
			else
			{
				detectedLinePointsVL.clear();
				SendLock = tFalse;
			}
			//TransmitStopline(tFalse, inputTimestamp);

			///
			straightness = 0;
			average = 0;
			if (detectedLinePointsHR.size() > 2*m_filterProperties.detectionLines / 3 && !SendLock)
			{
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsHR.begin(); Iter != detectedLinePointsHR.end(); Iter++)
				{
					average += Iter->GetY();
				}
				average /= detectedLinePointsHR.size();
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsHR.begin(); Iter != detectedLinePointsHR.end(); Iter++)
				{
					straightness += (average - Iter->GetY())*(average - Iter->GetY());
				}
				if (straightness <= m_filterProperties.StraightnessThreshold )
				{
					CrossPointHR = tTrue;
					// cout << "CrossPoint HR gefunden" << endl;
					SendLock = tTrue;
				}
				else
				{
					detectedLinePointsHR.clear();
				}
				straightness = 0;
			}
			else
			{
				detectedLinePointsHR.clear();
				SendLock = tFalse;
			}
			//TransmitStopline(tFalse, inputTimestamp);

			///
			straightness = 0;
			average = 0;
			if (detectedLinePointsVR.size() > 2*m_filterProperties.detectionLines / 3 && !SendLock)
			{
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsVR.begin(); Iter != detectedLinePointsVR.end(); Iter++)
				{
					average += Iter->GetY();
				}
				average /= detectedLinePointsVR.size();
				for (vector<cPoint>::const_iterator Iter = detectedLinePointsVR.begin(); Iter != detectedLinePointsVR.end(); Iter++)
				{
					straightness += (average - Iter->GetY())*(average - Iter->GetY());
				}
				if (straightness <= m_filterProperties.StraightnessThreshold )
				{
					CrossPointVR = tTrue;
					// cout << "CrossPoint VR gefunden" << endl;
					SendLock = tTrue;
				}
				else
				{
					detectedLinePointsVR.clear();
				}
				straightness = 0;
			}
			else
			{
				detectedLinePointsVR.clear();
				SendLock = tFalse;
			}
			//TransmitStopline(tFalse, inputTimestamp);

			// if all crossing points detected
			if(CrossPointHL && CrossPointVL && CrossPointHR && CrossPointVR)
			{
				TransmitStopline(tTrue, inputTimestamp);

				CrossPointHL = tFalse;
				CrossPointVL = tFalse;
				CrossPointHR = tFalse;
				CrossPointVR = tFalse;
			}

		}
		pSample->Unlock(l_pSrcBuffer);
	}

	// this way transmitting gcl output is less complex. merge line and point vectors
	detectedLinePoints = detectedLinePointsHL;
	detectedLinePoints.insert(detectedLinePoints.end(),detectedLinePointsVL.begin(),detectedLinePointsVL.end());
	detectedLinePoints.insert(detectedLinePoints.end(),detectedLinePointsHR.begin(),detectedLinePointsHR.end());
	detectedLinePoints.insert(detectedLinePoints.end(),detectedLinePointsVR.begin(),detectedLinePointsVR.end());

	if (!outputImage.empty() && m_oVideoOutputPin.IsConnected())
	{
		UpdateOutputImageFormat(outputImage);

		//create a cImage from CV Matrix
		cImage newImage;
		newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

		//create the new media sample
		cObjectPtr<IMediaSample> pMediaSample;
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
		//updating media sample
		RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
		//transmitting
		RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

		outputImage.release();
	}
	if (m_oGCLOutputPin.IsConnected()) transmitGCL(detectionLinesHL, detectionLinesVL, detectionLinesHR, detectionLinesVR, detectedLinePoints);

	RETURN_NOERROR;
}


tResult CrossroadDetector::findLinePointsHL(const vector<tInt>& detectionLinesHL, const cv::Mat& image, vector<cPoint>& detectedLinePointsHL, tTimeStamp inputTimestamp)
{
	//tFloat32 steeringAngle = 0;
	//iterate through the calculated horizontal lines
	for (vector<tInt>::const_iterator nline = detectionLinesHL.begin(); nline != detectionLinesHL.end(); nline++)
	{
		uchar ucLastVal = 0;

		// create vector with line data
		const uchar* p = image.ptr<uchar>(*nline, m_filterProperties.ROIOffsetYHL);
		std::vector<uchar> lineData(p, p + m_filterProperties.ROIHeightH);

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
					detectedLinePointsHL.push_back(cPoint(*nline,tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
																	  m_filterProperties.ROIOffsetYHL)));

					detectedStartCornerLine = tFalse;
					columnStartCornerLine = 0;
					break;
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


tResult CrossroadDetector::findLinePointsVL(const vector<tInt>& detectionLines, const cv::Mat& image, vector<cPoint>& detectedLinePointsVL, tTimeStamp inputTimestamp)
{
	//tFloat32 steeringAngle = 0;
	//iterate through the calculated horizontal lines
	for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
	{
		uchar ucLastVal = 0;

		// create vector with line data
		const uchar* p = image.ptr<uchar>(*nline, m_filterProperties.ROIOffsetXVL);
		std::vector<uchar> lineData(p, p + m_filterProperties.ROIWidthV);

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
					detectedLinePointsVL.push_back(cPoint(tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
															   m_filterProperties.ROIOffsetXVL), *nline));

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


tResult CrossroadDetector::findLinePointsHR(const vector<tInt>& detectionLines, const cv::Mat& image, vector<cPoint>& detectedLinePointsHR, tTimeStamp inputTimestamp)
{
	//tFloat32 steeringAngle = 0;
	//iterate through the calculated horizontal lines
	for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
	{
		uchar ucLastVal = 0;

		// create vector with line data
		const uchar* p = image.ptr<uchar>(*nline, m_filterProperties.ROIOffsetYHR);
		std::vector<uchar> lineData(p, p + m_filterProperties.ROIHeightH);

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
					detectedLinePointsHR.push_back(cPoint(*nline,tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
																	  m_filterProperties.ROIOffsetYHR)));

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

tResult CrossroadDetector::findLinePointsVR(const vector<tInt>& detectionLines, const cv::Mat& image, vector<cPoint>& detectedLinePointsVR, tTimeStamp inputTimestamp)
{
	//tFloat32 steeringAngle = 0;
	//iterate through the calculated horizontal lines
	for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
	{
		uchar ucLastVal = 0;

		// create vector with line data
		const uchar* p = image.ptr<uchar>(*nline, m_filterProperties.ROIOffsetXVR);
		std::vector<uchar> lineData(p, p + m_filterProperties.ROIWidthV);

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
					detectedLinePointsVR.push_back(cPoint(tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
															   m_filterProperties.ROIOffsetXVR), *nline));

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


tResult CrossroadDetector::getDetectionLinesHL(vector<tInt>& detectionLines)
{
	tInt distanceBetweenDetectionLines = m_filterProperties.ROIWidthH / (m_filterProperties.detectionLines + 1);

	for (uint i = 1; i <= m_filterProperties.detectionLines; i++)
	{
		detectionLines.push_back(m_filterProperties.ROIOffsetXHL + i * distanceBetweenDetectionLines);
	}
	RETURN_NOERROR;
}

tResult CrossroadDetector::getDetectionLinesVL(vector<tInt>& detectionLines)
{
	tInt distanceBetweenDetectionLines = m_filterProperties.ROIHeightV / (m_filterProperties.detectionLines + 1);

	for (uint i = 1; i <= m_filterProperties.detectionLines; i++)
	{
		detectionLines.push_back(m_filterProperties.ROIOffsetYVL + i * distanceBetweenDetectionLines);
	}
	RETURN_NOERROR;
}

tResult CrossroadDetector::getDetectionLinesHR(vector<tInt>& detectionLines)
{
	tInt distanceBetweenDetectionLines = m_filterProperties.ROIWidthH / (m_filterProperties.detectionLines + 1);

	for (uint i = 1; i <= m_filterProperties.detectionLines; i++)
	{
		detectionLines.push_back(m_filterProperties.ROIOffsetXHR + i * distanceBetweenDetectionLines);
	}
	RETURN_NOERROR;
}

tResult CrossroadDetector::getDetectionLinesVR(vector<tInt>& detectionLines)
{
	tInt distanceBetweenDetectionLines = m_filterProperties.ROIHeightV / (m_filterProperties.detectionLines + 1);

	for (uint i = 1; i <= m_filterProperties.detectionLines; i++)
	{
		detectionLines.push_back(m_filterProperties.ROIOffsetYVR + i * distanceBetweenDetectionLines);
	}
	RETURN_NOERROR;
}


tResult CrossroadDetector::UpdateInputImageFormat(const tBitmapFormat* pFormat)
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

tResult CrossroadDetector::transmitGCL(const vector<tInt>& detectionLinesHL, const vector<tInt>& detectionLinesVL, const vector<tInt>& detectionLinesHR, const vector<tInt>& detectionLinesVR, const vector<cPoint>& detectedLinePoints)
{

	IDynamicMemoryBlock* pGCLCmdDebugInfo;

	cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);

	//set color
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Red.GetRGBA());
	//show roi
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXHL, m_filterProperties.ROIOffsetYHL,
							 m_filterProperties.ROIOffsetXHL + m_filterProperties.ROIWidthH, m_filterProperties.ROIOffsetYHL + m_filterProperties.ROIHeightH);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXVL, m_filterProperties.ROIOffsetYVL,
							 m_filterProperties.ROIOffsetXVL + m_filterProperties.ROIWidthV, m_filterProperties.ROIOffsetYVL + m_filterProperties.ROIHeightV);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXHR, m_filterProperties.ROIOffsetYHR,
							 m_filterProperties.ROIOffsetXHR + m_filterProperties.ROIWidthH, m_filterProperties.ROIOffsetYHR + m_filterProperties.ROIHeightH);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXVR, m_filterProperties.ROIOffsetYVR,
							 m_filterProperties.ROIOffsetXVR + m_filterProperties.ROIWidthV, m_filterProperties.ROIOffsetYVR + m_filterProperties.ROIHeightV);

	//show detection lines
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Yellow.GetRGBA());
	for (vector<tInt>::const_iterator it = detectionLinesHL.begin(); it != detectionLinesHL.end(); it++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE,*it, m_filterProperties.ROIOffsetYHL,*it , m_filterProperties.ROIOffsetYHL + m_filterProperties.ROIHeightH);
	}
	for (vector<tInt>::const_iterator it = detectionLinesVL.begin(); it != detectionLinesVL.end(); it++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE, m_filterProperties.ROIOffsetXVL,*it , m_filterProperties.ROIOffsetXVL + m_filterProperties.ROIWidthV,*it);
	}
	for (vector<tInt>::const_iterator it = detectionLinesHR.begin(); it != detectionLinesHR.end(); it++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE,*it, m_filterProperties.ROIOffsetYHR,*it , m_filterProperties.ROIOffsetYHR + m_filterProperties.ROIHeightH);
	}
	for (vector<tInt>::const_iterator it = detectionLinesVR.begin(); it != detectionLinesVR.end(); it++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE, m_filterProperties.ROIOffsetXVR,*it , m_filterProperties.ROIOffsetXVR + m_filterProperties.ROIWidthV,*it);
	}

	//show detected lane points
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Green.GetRGBA());
	for (vector<cPoint>::const_iterator LinePointsIter = detectedLinePoints.begin(); LinePointsIter != detectedLinePoints.end(); LinePointsIter++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE, LinePointsIter->GetX(), LinePointsIter->GetY(), 20);
	}

	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Blue.GetRGBA());
	//show minimum and maximum line width
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXHL, m_filterProperties.ROIOffsetYHL,
							 m_filterProperties.ROIOffsetXHL + m_filterProperties.maxLineWidth, m_filterProperties.ROIOffsetYHL - 20);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXHL + m_filterProperties.ROIWidthH - m_filterProperties.maxLineWidth, m_filterProperties.ROIOffsetYHL,
							 m_filterProperties.ROIOffsetXHL + m_filterProperties.ROIWidthH, m_filterProperties.ROIOffsetYHL - 20);


	//show minimum and maximum line width at left and right roi
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXHL + m_filterProperties.ROIWidthH - m_filterProperties.minLineWidth, m_filterProperties.ROIOffsetYHL - 20,
							 m_filterProperties.ROIOffsetXHL + m_filterProperties.ROIWidthH, m_filterProperties.ROIOffsetYHL - 40);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetXHL, m_filterProperties.ROIOffsetYHL - 20,
							 m_filterProperties.ROIOffsetXHL + m_filterProperties.minLineWidth, m_filterProperties.ROIOffsetYHL - 40);


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

tResult CrossroadDetector::UpdateOutputImageFormat(const cv::Mat& outputImage)
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

tResult CrossroadDetector::TransmitStopline(const tBool Crossroad, tTimeStamp timestamp)
{
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
		pCoderOutput->Set(m_szIDOutputBoolValue, (tVoid*)&Crossroad);
		pCoderOutput->Set(m_szIDOutputBoolTimeStamp, (tVoid*)&timestamp);
	}

	pMediaSample->SetTime(timestamp);
	RETURN_IF_FAILED(m_oBoolValuePin.Transmit(pMediaSample));
	RETURN_NOERROR;
}
