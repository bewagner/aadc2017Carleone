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
#include "LaneSteeringBeta.h"
#include <iostream>

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, LaneSteeringBeta)



LaneSteeringBeta::LaneSteeringBeta(const tChar* __info) : cFilter(__info)
{
	SetPropertyBool("DEBUG::DebutOutput", tFalse);
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("DEBUG::DebutOutput", tFalse);
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
	m_szIDsOutputSteeringSet = tFalse;
	timeofframe = 0;
	cumulatedDifference = 0.0f;
	SteeringValue = 0.0f;
	LastSteeringValue = 0.0f;
	dynWidthL = 0;
	dynWidthR = 0;
	ResetSteering = tFalse;
	ResetSet = tFalse;
	DistanceBufferIDSet = tFalse;
	SwitchLaneIDSet = tFalse;
	DistanceAtStart = 0.0f;
	DistanceDriven = 0.0f;
	RightLane = tTrue;
	OverallDistanceSet = tTrue;
	SwitchToRightLane = tFalse;
	SwitchToLeftLane = tFalse;
	ROISwitched = tFalse;
	LastSteering = 0.0f;
	WobbleIDSet = tFalse;


    DistanceWobbleSet = tFalse;
    Wobbling = tFalse;
    DistanceAtWobbleStart = 0.0f;;
    DistanceDrivenWobble = 0.0f;

	SetPropertyFloat("LANESWITCH::SteeringOffsetRL", 60.0f);
	SetPropertyStr("LANESWITCH::SteeringOffsetRL" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("LANESWITCH::SteeringOffsetRL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("LANESWITCH::LaneSwitchDistanceRL", 2.0f);
	SetPropertyStr("LANESWITCH::LaneSwitchDistanceRL" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("LANESWITCH::LaneSwitchDistanceRL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("LANESWITCH::ROISwitchRL", 100);
	SetPropertyStr("LANESWITCH::ROISwitchRL" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("LANESWITCH::ROISwitchRL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("LANESWITCH::SteeringOffsetLR", 40.0f);
	SetPropertyStr("LANESWITCH::SteeringOffsetLR" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("LANESWITCH::SteeringOffsetLR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("LANESWITCH::LaneSwitchDistanceLR", 2.0f);
	SetPropertyStr("LANESWITCH::LaneSwitchDistanceLR" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("LANESWITCH::LaneSwitchDistanceLR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("LANESWITCH::ROISwitchLR", 100);
	SetPropertyStr("LANESWITCH::ROISwitchLR" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("LANESWITCH::ROISwitchLR" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyFloat("STE::Offset", 0.0f);
	SetPropertyStr("STE::Offset" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("STE::Offset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("STE::SteeringMultiplier", 0.5);
	SetPropertyStr("STE::SteeringMultiplier" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output");
	SetPropertyBool("STE::SteeringMultiplier" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("STE::SteeringMultiplierR", 1.5);
	SetPropertyStr("STE::SteeringMultiplierR" NSSUBPROP_DESCRIPTION, "Scaling Factor for Steering Output Right");
	SetPropertyBool("STE::SteeringMultiplierR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("STE::UpperBound", 200);
	SetPropertyStr("STE::UpperBound" NSSUBPROP_DESCRIPTION, "Upper Bound for cumulated Difference");
	SetPropertyBool("STE::UpperBound" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::XOffsetR", 675);
	SetPropertyStr("ROI::XOffsetR" NSSUBPROP_DESCRIPTION, "X Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::XOffsetR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::YOffsetR", 685);
	SetPropertyStr("ROI::YOffsetR" NSSUBPROP_DESCRIPTION, "Y Offset for RIght Region of Interest Rectangular");
	SetPropertyBool("ROI::YOffsetR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::WidthR", 300);
	SetPropertyStr("ROI::WidthR" NSSUBPROP_DESCRIPTION, "Width of the Region of Interest Rectangular");
	SetPropertyBool("ROI::WidthR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::HeightR", 30);
	SetPropertyStr("ROI::HeightR" NSSUBPROP_DESCRIPTION, "Height of the Region of Interest Rectangular");
	SetPropertyBool("ROI::HeightR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::XOffsetL", 255);
	SetPropertyStr("ROI::XOffsetL" NSSUBPROP_DESCRIPTION, "X Offset for Right Region of Interest Rectangular");
	SetPropertyBool("ROI::XOffsetL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::YOffsetL", 685);
	SetPropertyStr("ROI::YOffsetL" NSSUBPROP_DESCRIPTION, "Y Offset for RIght Region of Interest Rectangular");
	SetPropertyBool("ROI::YOffsetL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::WidthL", 300);
	SetPropertyStr("ROI::WidthL" NSSUBPROP_DESCRIPTION, "Width of the Region of Interest Rectangular");
	SetPropertyBool("ROI::WidthL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::HeightL", 30);
	SetPropertyStr("ROI::HeightL" NSSUBPROP_DESCRIPTION, "Height of the Region of Interest Rectangular");
	SetPropertyBool("ROI::HeightL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::DynamicWidthL",160);
	SetPropertyStr("ROI::DynamicWidthL" NSSUBPROP_DESCRIPTION, "Maximum Dnymaic Width of left ROI");
	SetPropertyBool("ROI::DynamicWidthL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::DynamicWidthR",160);
	SetPropertyStr("ROI::DynamicWidthR" NSSUBPROP_DESCRIPTION, "Maximum Dnymaic Width of Right ROI");
	SetPropertyBool("ROI::DynamicWidthR" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::DynamicWidthMultiplier",10);
	SetPropertyStr("ROI::DynamicWidthMultiplier" NSSUBPROP_DESCRIPTION, "Multiplier of dynamic ROI-Width");
	SetPropertyBool("ROI::DynamicWidthMultiplier" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Algorithm::Detection Lines", 25);
	SetPropertyStr("Algorithm::Detection Lines" NSSUBPROP_DESCRIPTION, "number of detection lines searched in ROI");
	SetPropertyBool("Algorithm::Detection Lines" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Detection Lines" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Minimum Line Width", 8);
	SetPropertyStr("Algorithm::Minimum Line Width" NSSUBPROP_DESCRIPTION, "Minimum Line Width in Pixel");
	SetPropertyBool("Algorithm::Minimum Line Width" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Minimum Line Width" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Maximum Line Width", 55);
	SetPropertyStr("Algorithm::Maximum Line Width" NSSUBPROP_DESCRIPTION, "Maximum Line Width in Pixel");
	SetPropertyBool("Algorithm::Maximum Line Width" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Algorithm::Maximum Line Width" NSSUBPROP_MIN, 1);

	SetPropertyInt("Algorithm::Minimum Line Contrast", 30);
	SetPropertyStr("Algorithm::Minimum Line Contrast" NSSUBPROP_DESCRIPTION, "Mimimum line contrast in gray Values");
	SetPropertyBool("Algorithm::Minimum Line Contrast" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MIN, 1);
	SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MAX, 255);

	SetPropertyInt("Algorithm::Image Binarization Threshold", 1);
	SetPropertyStr("Algorithm::Image Binarization Threshold" NSSUBPROP_DESCRIPTION, "Threshold for image binarization");
	SetPropertyBool("Algorithm::Image Binarization Threshold" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MIN, 1);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MAX, 255);

	SetPropertyInt("Reference::RefeRXU", 855);
	SetPropertyStr("Reference::RefeRXU" NSSUBPROP_DESCRIPTION, "X Coordinate of upper reference point");
	SetPropertyBool("Reference::RefeRXU" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Reference::RefeRYU",685);
	SetPropertyStr("Reference::RefeRYU" NSSUBPROP_DESCRIPTION, "Y Coordinate of upper reference point");
	SetPropertyBool("Reference::RefeRYU" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Reference::RefeRXL", 890);
	SetPropertyStr("Reference::RefeRXL" NSSUBPROP_DESCRIPTION, "X Coordinate of lower reference point");
	SetPropertyBool("Reference::RefeRXL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Reference::RefeRYL", 720);
	SetPropertyStr("Reference::RefeRYL" NSSUBPROP_DESCRIPTION, "Y Coordinate of lower reference point");
	SetPropertyBool("Reference::RefeRYL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Reference::RefeLXU", 430);
	SetPropertyStr("Reference::RefeLXU" NSSUBPROP_DESCRIPTION, "X Coordinate of upper reference point");
	SetPropertyBool("Reference::RefeLXU" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Reference::RefeLYU",685);
	SetPropertyStr("Reference::RefeLYU" NSSUBPROP_DESCRIPTION, "Y Coordinate of upper reference point");
	SetPropertyBool("Reference::RefeLYU" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Reference::RefeLXL", 400);
	SetPropertyStr("Reference::RefeLXL" NSSUBPROP_DESCRIPTION, "X Coordinate of lower reference point");
	SetPropertyBool("Reference::RefeLXL" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Reference::RefeLYL", 720);
	SetPropertyStr("Reference::RefeLYL" NSSUBPROP_DESCRIPTION, "Y Coordinate of lower reference point");
	SetPropertyBool("Reference::RefeLYL" NSSUBPROP_ISCHANGEABLE, tTrue);

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

	SetPropertyBool("ReverseMode", tFalse);
	SetPropertyStr("ReverseMode" NSSUBPROP_DESCRIPTION, "ReverseMode");
}

LaneSteeringBeta::~LaneSteeringBeta()
{
}

tResult LaneSteeringBeta::Start(__exception)
{

	return cFilter::Start(__exception_ptr);
}

tResult LaneSteeringBeta::Stop(__exception)
{
	//destroyWindow("Debug");
	return cFilter::Stop(__exception_ptr);
}
tResult LaneSteeringBeta::Init(tInitStage eStage, __exception)
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

		//Steering Output
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
		//mediatype für signalvalues
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0,0,0,"tSignalValue",strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValueTwo));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValueThree));
		// Bool
		//mediatype für signalvalues
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0,0,0,"tBoolSignalValue",strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		//MediaTypbeschreibungen

		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolSignalValue));


		// Pins Erstellen und registrieren
		RETURN_IF_FAILED(m_oOutputSteeringAngle.Create("SteeringOut",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputSteeringAngle));

		//create pin for output
		RETURN_IF_FAILED(ResetDynamicWidthIn.Create("ResetDynamicWidth",pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&ResetDynamicWidthIn));

		RETURN_IF_FAILED(DistanceIn.Create("DistanceInput",pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&DistanceIn));

		RETURN_IF_FAILED(SwitchLaneIn.Create("SwitchLane",pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&SwitchLaneIn));

		RETURN_IF_FAILED(WobbleIn.Create("Wobble",pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&WobbleIn));


	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageGraphReady)
	{
		Reverse = GetPropertyBool("ReverseMode",tFalse);
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



tResult LaneSteeringBeta::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	if (eStage == StageGraphReady)
	{
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult LaneSteeringBeta::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
		if (pSource == &ResetDynamicWidthIn)
		{
			RETURN_IF_FAILED(ProcessReset(pMediaSample));
		}
		else if (pSource == &DistanceIn)
		{
			RETURN_IF_FAILED(ProcessDistance(pMediaSample));
		}
		else if (pSource == &SwitchLaneIn)
		{
			RETURN_IF_FAILED(ProcessSwitchLane(pMediaSample));
		}
		else if(pSource == &WobbleIn) {
			RETURN_IF_FAILED(ProcessWobble(pMediaSample));
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

tResult LaneSteeringBeta::PropertyChanged(const tChar* strName)
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
	if (cString::IsEqual(strName, "ROI::WidthR"))
		m_filterProperties.ROIWidthR = GetPropertyInt("ROI::WidthR");
	else if (cString::IsEqual(strName, "ROI::HeightR"))
		m_filterProperties.ROIHeightR = GetPropertyInt("ROI::HeightR");
	else if (cString::IsEqual(strName, "ROI::XOffsetR"))
		m_filterProperties.ROIOffsetRX = GetPropertyInt("ROI::XOffsetR");
	else if (cString::IsEqual(strName, "ROI::YOffsetR"))
		m_filterProperties.ROIOffsetRY = GetPropertyInt("ROI::YOffsetR");
	else if (cString::IsEqual(strName, "ROI::WidthL"))
		m_filterProperties.ROIWidthL = GetPropertyInt("ROI::WidthL");
	else if (cString::IsEqual(strName, "ROI::HeightL"))
		m_filterProperties.ROIHeightL = GetPropertyInt("ROI::HeightL");
	else if (cString::IsEqual(strName, "ROI::XOffsetL"))
		m_filterProperties.ROIOffsetLX = GetPropertyInt("ROI::XOffsetL");
	else if (cString::IsEqual(strName, "ROI::YOffsetL"))
		m_filterProperties.ROIOffsetLY = GetPropertyInt("ROI::YOffsetL");

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

	else if (cString::IsEqual(strName, "STE::UpperBound"))
		m_filterProperties.UpperBound = GetPropertyInt("STE::UpperBound");

	else if (cString::IsEqual(strName, "Reference::RefeRXU"))
		m_filterProperties.RefeRXU = GetPropertyInt("Reference::RefeRXU");
	else if (cString::IsEqual(strName, "Reference::RefeRXL"))
		m_filterProperties.RefeRXL = GetPropertyInt("Reference::RefeRXL");
	else if (cString::IsEqual(strName, "Reference::RefeRYU"))
		m_filterProperties.RefeRYU = GetPropertyInt("Reference::RefeRYU");
	else if (cString::IsEqual(strName, "Reference::RefeRYL"))
		m_filterProperties.RefeRYL = GetPropertyInt("Reference::RefeRYL");
	else if (cString::IsEqual(strName, "Reference::RefeLXU"))
		m_filterProperties.RefeLXU = GetPropertyInt("Reference::RefeLXU");
	else if (cString::IsEqual(strName, "Reference::RefeLXL"))
		m_filterProperties.RefeLXL = GetPropertyInt("Reference::RefeLXL");
	else if (cString::IsEqual(strName, "Reference::RefeLYU"))
		m_filterProperties.RefeLYU = GetPropertyInt("Reference::RefeLYU");
	else if (cString::IsEqual(strName, "Reference::RefeLYL"))
		m_filterProperties.RefeLYL = GetPropertyInt("Reference::RefeLYL");

	else if (cString::IsEqual(strName, "STE::SteeringMultiplier"))
		m_filterProperties.SteeringMultiplier = GetPropertyFloat("STE::SteeringMultiplier");
	else if (cString::IsEqual(strName, "STE::SteeringMultiplierR"))
		m_filterProperties.SteeringMultiplierR = GetPropertyFloat("STE::SteeringMultiplierR");
	else if (cString::IsEqual(strName, "STE::Offset"))
		m_filterProperties.SteeringOffset = GetPropertyFloat("STE::Offset");

	else if (cString::IsEqual(strName, "Canny::Threshold"))
		m_filterProperties.Threshold = GetPropertyInt("Canny::Threshold");
	else if (cString::IsEqual(strName, "Canny::ThresholdRatio"))
		m_filterProperties.ThresholdRatio = GetPropertyInt("Canny::ThresholdRatio");
	else if (cString::IsEqual(strName, "Canny::KernelSize"))
		m_filterProperties.KernelSize = GetPropertyInt("Canny::KernelSize");

	else if (cString::IsEqual(strName, "Hough::AccumulatorThreshold"))
		m_filterProperties.AccumulatorThreshold = GetPropertyInt("Hough::AccumulatorThreshold");

	else if(cString::IsEqual(strName, "ROI::DynamicWidthL"))
		m_filterProperties.maxDynamicWidthL = GetPropertyInt("ROI::DynamicWidthL");
	else if(cString::IsEqual(strName, "ROI::DynamicWidthR"))
		m_filterProperties.maxDynamicWidthR = GetPropertyInt("ROI::DynamicWidthR");
	else if(cString::IsEqual(strName, "ROI::DynamicWidthMultiplier"))
		m_filterProperties.dynWidthMultiplier = GetPropertyInt("ROI::DynamicWidthMultiplier");
	else if(cString::IsEqual(strName, "LANESWITCH::SteeringOffsetLR"))
		m_filterProperties.LaneSwitchOffsetLR = GetPropertyFloat("LANESWITCH::SteeringOffsetLR");
	else if(cString::IsEqual(strName, "LANESWITCH::LaneSwitchDistanceLR"))
		m_filterProperties.LaneSwitchDistanceLR = GetPropertyFloat("LANESWITCH::LaneSwitchDistanceLR");
	else if(cString::IsEqual(strName, "LANESWITCH::ROISwitchLR"))
		m_filterProperties.LaneSwitchROIOffsetLR = GetPropertyInt("LANESWITCH::ROISwitchLR");
	else if(cString::IsEqual(strName, "LANESWITCH::SteeringOffsetRL"))
		m_filterProperties.LaneSwitchOffsetRL = GetPropertyFloat("LANESWITCH::SteeringOffsetRL");
	else if(cString::IsEqual(strName, "LANESWITCH::LaneSwitchDistanceRL"))
		m_filterProperties.LaneSwitchDistanceRL = GetPropertyFloat("LANESWITCH::LaneSwitchDistanceRL");
	else if(cString::IsEqual(strName, "LANESWITCH::ROISwitchRL"))
		m_filterProperties.LaneSwitchROIOffsetRL = GetPropertyInt("LANESWITCH::ROISwitchRL");

	RETURN_NOERROR;
}


tResult LaneSteeringBeta::ProcessVideo(IMediaSample* pSample, tTimeStamp inputTimestamp)
{
	RETURN_IF_POINTER_NULL(pSample);
	// new image for result
	cv::Mat outputImage;
	tFloat32 DiffLeft = 0.0f;

	// here we store the pixel lines in the image where we search for lanes
	vector<tInt> detectionLinesR;
	vector<tInt> detectionLinesL;
	vector<tInt> detectionLines;

	// here we have all the detected line points
	vector<cPoint> detectedLinePointsR;
	vector<cPoint> detectedLinePointsL;
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

			//initialize regions of interests, we are editing just the parts of the image that is of interest to us
			cv::Mat ROIimageR, ROIimageL;
			// define right ROI
			cv::Rect ROIR = Rect(m_filterProperties.ROIOffsetRX+dynWidthR,m_filterProperties.ROIOffsetRY,m_filterProperties.ROIWidthR,m_filterProperties.ROIHeightR);
			//get data of region of interest
			cv::Mat croppedRefR(outputImage, ROIR);

			// ...and safe it on our roi-images
			//croppedRefR.copyTo(ROIimageR);
			ROIimageR = outputImage;
			// convert to hard black and white
			//threshold(ROIimageR, ROIimageR, m_filterProperties.thresholdImageBinarization, 255, THRESH_TOZERO);// Generate Binary Image
			//copy edited image back into the original
			normalize(ROIimageR,ROIimageR, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
			//medianBlur(ROIimageR,ROIimageR,3);
			//ROIimageR.copyTo(outputImage(cv::Rect(m_filterProperties.ROIOffsetRX+dynWidthR,m_filterProperties.ROIOffsetRY,ROIimageR.cols, ROIimageR.rows)));
			//calculate the detectionlines in image
			getDetectionLinesR(detectionLinesR);
			findLinePointsR(detectionLinesR, outputImage, detectedLinePointsR, inputTimestamp);

			cumulatedDifference = 0.0f;
			for (std::vector<cPoint>::iterator pointIterator = detectedLinePointsR.begin(); pointIterator != detectedLinePointsR.end(); pointIterator++)
			{
				cPoint P = *pointIterator;
				tFloat32 x = (tFloat32)P.x;
				tFloat32 y = (tFloat32)P.y;
				tFloat32 lambda = (y - m_filterProperties.RefeRYU)/(m_filterProperties.RefeRYL - m_filterProperties.RefeRYU);
				tFloat32 refx = m_filterProperties.RefeRXU + lambda*(m_filterProperties.RefeRXL -m_filterProperties.RefeRXU);
				cumulatedDifference += refx -x;
			}

				//as above, for the left roi
				cv::Rect ROIL = Rect(m_filterProperties.ROIOffsetLX -dynWidthL,m_filterProperties.ROIOffsetLY,m_filterProperties.ROIWidthL,m_filterProperties.ROIHeightL);
				cv::Mat croppedRefL(outputImage, ROIL);
				//croppedRefL.copyTo(ROIimageL);
				ROIimageL = outputImage;
				//threshold(ROIimageL, ROIimageL, m_filterProperties.thresholdImageBinarization, 255, THRESH_TOZERO);// Generate Binary Image
				//copy edited image back into the original
				normalize(ROIimageL,ROIimageL, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
				//medianBlur(ROIimageL,ROIimageL,3);
				//ROIimageL.copyTo(outputImage(cv::Rect(m_filterProperties.ROIOffsetLX-dynWidthL,m_filterProperties.ROIOffsetLY,ROIimageL.cols, ROIimageL.rows)));
				getDetectionLinesL(detectionLinesL);
				findLinePointsL(detectionLinesL, outputImage, detectedLinePointsL, inputTimestamp);
				for (std::vector<cPoint>::iterator pointIterator = detectedLinePointsL.begin(); pointIterator != detectedLinePointsL.end(); pointIterator++)
				{
					cPoint P = *pointIterator;
					tFloat32 x = (tFloat32)P.x;
					tFloat32 y = (tFloat32)P.y;
					tFloat32 lambda = ((tFloat32)y - m_filterProperties.RefeRYU)/(m_filterProperties.RefeRYL - m_filterProperties.RefeRYU);
					tFloat32 refx = m_filterProperties.RefeLXU + lambda*(m_filterProperties.RefeLXL -m_filterProperties.RefeLXU);
					cumulatedDifference += refx -x;
					//DiffLeft += refx - x;
				}
		}
		pSample->Unlock(l_pSrcBuffer);
	}
	//now we're calculating the average offset between our current lateral position and the reference lines


	//if no line is detected, do not steer
	if (!(detectedLinePointsR.empty() && detectedLinePointsL.empty()))
	{
		cumulatedDifference /= (tFloat32)(detectedLinePointsR.size() + detectedLinePointsL.size());
		//cumulatedDifference /= (m_filterProperties.ROIWidthL + m_filterProperties.ROIWidthR)/2;
		LastSteeringValue = cumulatedDifference;
	}
	else
	{
		cumulatedDifference = LastSteeringValue;
	}

	if (cumulatedDifference < 0){
		SteeringValue = -(m_filterProperties.SteeringMultiplierR)*cumulatedDifference/m_filterProperties.UpperBound*50;
		//DiffLeft = -(m_filterProperties.SteeringMultiplierR)*DiffLeft/m_filterProperties.UpperBound*50;

	}
	else
	{
		SteeringValue = -(m_filterProperties.SteeringMultiplier)*cumulatedDifference/m_filterProperties.UpperBound*50;
		//DiffLeft = -(m_filterProperties.SteeringMultiplier)*DiffLeft/m_filterProperties.UpperBound*50;

	}
	if (abs(SteeringValue) > 100) SteeringValue = SteeringValue/abs(SteeringValue)*100.0f;
	//if (abs(DiffLeft) > 100) DiffLeft = DiffLeft/abs(DiffLeft)*100.0f;


	if (!ResetSteering)
	{
		dynWidthR = m_filterProperties.dynWidthMultiplier*(tInt32)SteeringValue;
		dynWidthL = -m_filterProperties.dynWidthMultiplier*(tInt32)SteeringValue;
	}
	else {
		ResetSteering = tFalse;
		SteeringValue = 0;
	}
	if(abs(dynWidthR) > m_filterProperties.maxDynamicWidthR) dynWidthR = dynWidthR > 0 ? m_filterProperties.maxDynamicWidthR : - m_filterProperties.maxDynamicWidthR;
	//dynWidthL = -dynWidthR;
	if(abs(dynWidthL) > m_filterProperties.maxDynamicWidthL) dynWidthL = dynWidthL > 0 ? m_filterProperties.maxDynamicWidthL : - m_filterProperties.maxDynamicWidthL;
	SteeringValue += m_filterProperties.SteeringOffset;
	SteeringValue = abs(SteeringValue + m_filterProperties.SteeringOffset) > 100 ? SteeringValue : SteeringValue + m_filterProperties.SteeringOffset;

	if (SwitchToRightLane){
		if (DistanceDriven < m_filterProperties.LaneSwitchDistanceLR / 2.0f) {
			SteeringValue += m_filterProperties.LaneSwitchOffsetLR*DistanceDriven / (m_filterProperties.LaneSwitchDistanceLR / 2.0f);
			//tFloat32 off = m_filterProperties.LaneSwitchOffsetLR*DistanceDriven / (m_filterProperties.LaneSwitchDistanceLR / 2.0f);
			//LOG_INFO(cString::Format("Offset: %f, DistanceDriven %f", off, DistanceDriven));
		}
		else if (DistanceDriven < m_filterProperties.LaneSwitchDistanceLR) {
			SteeringValue += m_filterProperties.LaneSwitchOffsetLR;
		}
		else {
			if(!ROISwitched) {
				dynWidthR = m_filterProperties.LaneSwitchROIOffsetLR;
				dynWidthL = -m_filterProperties.LaneSwitchROIOffsetLR;
				ROISwitched = tTrue;
				SteeringValue += m_filterProperties.LaneSwitchOffsetLR;
			}
			//SteeringValue -= m_filterProperties.LaneSwitchOffset*(m_filterProperties.LaneSwitchDistance - DistanceDriven) / (m_filterProperties.LaneSwitchDistance / 2.0f);
		}
	}
	else if (SwitchToLeftLane){
		if (DistanceDriven < m_filterProperties.LaneSwitchDistanceRL / 2.0f) {
			SteeringValue -= m_filterProperties.LaneSwitchOffsetRL*DistanceDriven / (m_filterProperties.LaneSwitchDistanceRL / 2.0f);
		}
		else if (DistanceDriven < m_filterProperties.LaneSwitchDistanceRL){
			SteeringValue -= m_filterProperties.LaneSwitchOffsetRL;
		}
		else {
			if(!ROISwitched) {
				dynWidthR = m_filterProperties.LaneSwitchROIOffsetRL;
				dynWidthL = -m_filterProperties.LaneSwitchROIOffsetRL;
				ROISwitched = tTrue;
				SteeringValue -= m_filterProperties.LaneSwitchOffsetRL;
			}
			//SteeringValue += m_filterProperties.LaneSwitchOffset*(m_filterProperties.LaneSwitchDistance - DistanceDriven) / (m_filterProperties.LaneSwitchDistance / 2.0f);
		}
	}

	if (Wobbling) {
		SteeringValue += m_filterProperties.KernelSize*sin(20.0f*DistanceDrivenWobble);
		if(DistanceDrivenWobble > 2*m_filterProperties.LaneSwitchDistanceRL){
			Wobbling = tFalse;
		}
	}

	if(DistanceDriven > m_filterProperties.LaneSwitchDistanceRL || DistanceDriven > m_filterProperties.LaneSwitchDistanceLR) {
		SwitchToLeftLane = tFalse;
		SwitchToRightLane = tFalse;
		ROISwitched = tFalse;
		DistanceDriven = 0.0f;
	}
	LastSteering = SteeringValue;
	RETURN_IF_FAILED(TransmitSteering(SteeringValue, inputTimestamp));

	// these are pretty much useless, but just in case...

	//just for the looks and to tune the reference lines
	vector<cPoint> referencePoints;
	referencePoints.push_back(cPoint(m_filterProperties.RefeRXU,m_filterProperties.RefeRYU));
	referencePoints.push_back(cPoint(m_filterProperties.RefeRXL,m_filterProperties.RefeRYL));
	referencePoints.push_back(cPoint(m_filterProperties.RefeLXU,m_filterProperties.RefeLYU));
	referencePoints.push_back(cPoint(m_filterProperties.RefeLXL,m_filterProperties.RefeLYL));

	// this way transmitting gcl output is less complex. merge line and point vectors
	detectionLines = detectionLinesL;
	detectionLines.insert(detectionLines.end(),detectionLinesR.begin(),detectionLinesR.end());
	detectedLinePoints = detectedLinePointsR;
	detectedLinePoints.insert(detectedLinePoints.end(),detectedLinePointsL.begin(),detectedLinePointsL.end());

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

	if (m_oGCLOutputPin.IsConnected()) transmitGCL(detectionLines, detectedLinePoints, referencePoints);

	RETURN_NOERROR;
}


tResult LaneSteeringBeta::findLinePointsR(const vector<tInt>& detectionLines, const cv::Mat& image, vector<cPoint>& detectedLinePointsR, tTimeStamp inputTimestamp)
{
	//tFloat32 steeringAngle = 0;
	//iterate through the calculated horizontal lines
	for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
	{
		uchar ucLastVal = 0;

		// create vector with line data
		const uchar* p = image.ptr<uchar>(*nline, m_filterProperties.ROIOffsetRX + dynWidthR);
		std::vector<uchar> lineData(p, p + m_filterProperties.ROIWidthR);

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
					detectedLinePointsR.push_back(cPoint(tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
							m_filterProperties.ROIOffsetRX+ dynWidthR), *nline));

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


tResult LaneSteeringBeta::findLinePointsL(const vector<tInt>& detectionLines, const cv::Mat& image, vector<cPoint>& detectedLinePointsR, tTimeStamp inputTimestamp)
{
	//tFloat32 steeringAngle = 0;
	//iterate through the calculated horizontal lines
	for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
	{
		uchar ucLastVal = 0;

		// create vector with line data
		const uchar* p = image.ptr<uchar>(*nline, m_filterProperties.ROIOffsetLX-dynWidthL);
		std::vector<uchar> lineData(p, p + m_filterProperties.ROIWidthL);

		tBool detectedStartCornerLine = tFalse;
		tInt columnStartCornerLine = 0;

		for (std::vector<uchar>::iterator lineIterator = lineData.end(); lineIterator != lineData.begin(); lineIterator--)
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
					detectedLinePointsR.push_back(cPoint(tInt(currentIndex + abs(columnStartCornerLine - currentIndex) / 2 +
							m_filterProperties.ROIOffsetLX-dynWidthL), *nline));

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


tResult LaneSteeringBeta::getDetectionLinesR(vector<tInt>& detectionLines)
{
	tInt distanceBetweenDetectionLines = m_filterProperties.ROIHeightR / (m_filterProperties.detectionLines + 1);

	for (int i = 1; i <= m_filterProperties.detectionLines; i++)
	{
		detectionLines.push_back(m_filterProperties.ROIOffsetRY + i * distanceBetweenDetectionLines);
	}
	RETURN_NOERROR;
}

tResult LaneSteeringBeta::getDetectionLinesL(vector<tInt>& detectionLines)
{
	tInt distanceBetweenDetectionLines = m_filterProperties.ROIHeightL / (m_filterProperties.detectionLines + 1);

	for (int i = 1; i <= m_filterProperties.detectionLines; i++)
	{
		detectionLines.push_back(m_filterProperties.ROIOffsetLY + i * distanceBetweenDetectionLines);
	}
	RETURN_NOERROR;
}


tResult LaneSteeringBeta::UpdateInputImageFormat(const tBitmapFormat* pFormat)
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

tResult LaneSteeringBeta::transmitGCL(const vector<tInt>& detectionLines, const vector<cPoint>& detectedLinePoints, const vector<cPoint>& referencePoints)
{

	IDynamicMemoryBlock* pGCLCmdDebugInfo;

	cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);

	//set color
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Red.GetRGBA());
	//show roi
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetRX +dynWidthR, m_filterProperties.ROIOffsetRY,
			m_filterProperties.ROIOffsetRX + m_filterProperties.ROIWidthR +dynWidthR, m_filterProperties.ROIOffsetRY + m_filterProperties.ROIHeightR);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetLX+dynWidthR, m_filterProperties.ROIOffsetLY,
			m_filterProperties.ROIOffsetLX, m_filterProperties.ROIOffsetLY + m_filterProperties.ROIHeightL);



	//show detection lines
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Yellow.GetRGBA());
	for (vector<tInt>::const_iterator it = detectionLines.begin(); it != detectionLines.end(); it++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE, m_filterProperties.ROIOffsetRX+dynWidthR, *it, m_filterProperties.ROIOffsetRX + m_filterProperties.ROIWidthR+dynWidthR, *it);
	}
	for (vector<tInt>::const_iterator it = detectionLines.begin(); it != detectionLines.end(); it++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE, m_filterProperties.ROIOffsetLX+dynWidthR, *it, m_filterProperties.ROIOffsetLX + m_filterProperties.ROIWidthL+dynWidthR, *it);
	}

	//show detected lane points
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Green.GetRGBA());
	for (vector<cPoint>::const_iterator LinePointsIter = detectedLinePoints.begin(); LinePointsIter != detectedLinePoints.end(); LinePointsIter++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE, LinePointsIter->GetX(), LinePointsIter->GetY(), 20);
	}
	//show reference points
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Red.GetRGBA());
	for (vector<cPoint>::const_iterator RefPointsIter = referencePoints.begin(); RefPointsIter != referencePoints.end(); RefPointsIter++)
	{
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE, RefPointsIter->GetX(), RefPointsIter->GetY(), 10);
	}
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL, cColor::Blue.GetRGBA());
	//show minimum and maximum line width
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetRX, m_filterProperties.ROIOffsetRY,
			m_filterProperties.ROIOffsetRX + m_filterProperties.maxLineWidth, m_filterProperties.ROIOffsetRY - 20);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetRX + m_filterProperties.ROIWidthR - m_filterProperties.maxLineWidth, m_filterProperties.ROIOffsetRY,
			m_filterProperties.ROIOffsetRX + m_filterProperties.ROIWidthR, m_filterProperties.ROIOffsetRY - 20);


	//show minimum and maximum line width at left and right roi
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetRX + m_filterProperties.ROIWidthR - m_filterProperties.minLineWidth, m_filterProperties.ROIOffsetRY - 20,
			m_filterProperties.ROIOffsetRX + m_filterProperties.ROIWidthR, m_filterProperties.ROIOffsetRY - 40);
	cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetRX, m_filterProperties.ROIOffsetRY - 20,
			m_filterProperties.ROIOffsetRX + m_filterProperties.minLineWidth, m_filterProperties.ROIOffsetRY - 40);


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

tResult LaneSteeringBeta::UpdateOutputImageFormat(const cv::Mat& outputImage)
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

tResult LaneSteeringBeta::TransmitSteering(tFloat32 steering, tTimeStamp timestamp)
{
	__synchronized_obj(CritSteeringOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderOutput);
		if (!m_szIDsOutputSteeringSet)
		{
			pCoderOutput->GetID("f32Value",m_szIDOutputSteeringValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDOutputSteeringTimeStamp);
			m_szIDsOutputSteeringSet = tTrue;
		}
		//tFloat32 value = 0.0f;
		pCoderOutput->Set(m_szIDOutputSteeringValue, (tVoid*)&steering);
		pCoderOutput->Set(m_szIDOutputSteeringTimeStamp, (tVoid*)&timestamp);

		pMediaSample->SetTime(timestamp);
	}

	RETURN_IF_FAILED(m_oOutputSteeringAngle.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult LaneSteeringBeta::ProcessReset(IMediaSample* pSample)
{

	__synchronized_obj(CritDynWidthIn);

	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValueTwo, pSample, pCoderInput);
		tFloat32 value = 0.0f;
		tTimeStamp ts;
		if(!ResetSet)
		{
			pCoderInput->GetID("f32Value",BoolID);
			pCoderInput->GetID("ui32ArduinoTimestamp",BoolIDTs);
			ResetSet = tTrue;
		}

		pCoderInput->Get(BoolID, (tVoid*)&value);
		pCoderInput->Get(BoolIDTs, (tVoid*)&ts);
		dynWidthR = abs(value) > m_filterProperties.maxDynamicWidthR ? (value < 0 ? -m_filterProperties.maxDynamicWidthR : m_filterProperties.maxDynamicWidthR) : (tUInt32) value;
		dynWidthL = -dynWidthR;
		ResetSteering = tTrue;
	}

	RETURN_NOERROR;
}

tResult LaneSteeringBeta::ProcessDistance(IMediaSample* pSample)
{
	__synchronized_obj(CritDistanceIn);

	{
		// tTimeStamp timestamp;
		tFloat32 value = 0.0f;
			tTimeStamp ts;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValueThree, pSample, pCoderInput);

		if(!DistanceBufferIDSet)
		{
			pCoderInput->GetID("f32Value",DistanceBuffer);
			pCoderInput->GetID("ui32ArduinoTimestamp",DistanceBufferTs);
			DistanceBuffer = tTrue;
		}

		pCoderInput->Get(DistanceBuffer, (tVoid*)&value);
		pCoderInput->Get(DistanceBufferTs, (tVoid*)&ts);
		if(!OverallDistanceSet) {
				DistanceAtStart = value;
				DistanceDriven = 0.0f;
				OverallDistanceSet = tTrue;
			}
			else DistanceDriven = (SwitchToRightLane || SwitchToLeftLane) ? (value - DistanceAtStart) : 0;

			if(!DistanceWobbleSet) {
				DistanceAtWobbleStart = value;
				DistanceDrivenWobble = 0.0f;
				DistanceWobbleSet = tTrue;
			}
			else DistanceDrivenWobble = Wobbling ? (value - DistanceAtWobbleStart) : 0;
	}


	RETURN_NOERROR;
}

tResult LaneSteeringBeta::ProcessSwitchLane(IMediaSample* pSample)
{
	__synchronized_obj(CritBoolIn);
	tBool value; //= tFalse;
	tTimeStamp ts;
	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolSignalValue, pSample, pCoderInput);

		if(!SwitchLaneIDSet)
		{
			pCoderInput->GetID("bValue",SwitchLaneBuffer);
			pCoderInput->GetID("ui32ArduinoTimestamp",SwitchLaneBufferTs);
			SwitchLaneIDSet = tTrue;
		}

		pCoderInput->Get(SwitchLaneBuffer, (tVoid*)&value);
		pCoderInput->Get(SwitchLaneBufferTs, (tVoid*)&ts);
	}
	//if(fabs(LastSteering) < 20) {
	 if (value) {
		SwitchToRightLane = tTrue;
		SwitchToLeftLane = tFalse;
		LOG_INFO(cString("SwitchToRightLane"));
	}
	else {
		SwitchToLeftLane = tTrue;
		SwitchToRightLane = tFalse;
		LOG_INFO(cString("SwitchToLeftLane"));
	}
	OverallDistanceSet = tFalse;
	DistanceDriven = 0.0f;
	//}
	/*
	if (value) {
		dynWidthL = -m_filterProperties.maxDynamicWidthL;
		dynWidthR = m_filterProperties.maxDynamicWidthR;
	}
	else {
		dynWidthL = m_filterProperties.maxDynamicWidthL;
		dynWidthR = -m_filterProperties.maxDynamicWidthR;
	} */
	RETURN_NOERROR;
}

tResult LaneSteeringBeta::ProcessWobble(IMediaSample* pSample)
{
	__synchronized_obj(CritWobbleIn);

		{
			// tTimeStamp timestamp;
			__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolSignalValue, pSample, pCoderInput);
			tBool value; //= tFalse;
			tTimeStamp ts;
			if(!WobbleIDSet)
			{
				pCoderInput->GetID("bValue",WobbleBuffer);
				pCoderInput->GetID("ui32ArduinoTimestamp",WobbleBufferTs);
				WobbleIDSet = tTrue;
			}

			pCoderInput->Get(WobbleBuffer, (tVoid*)&value);
			pCoderInput->Get(WobbleBufferTs, (tVoid*)&ts);
			if (value){
						DistanceWobbleSet = tFalse;
						Wobbling = tTrue;
						DistanceDrivenWobble = 0.0f;
					}
		}


		RETURN_NOERROR;
}
