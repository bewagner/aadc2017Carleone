#include <math.h>
#include "stdafx.h"
#include "EmergencyStop.h"
#include <iostream>
#include "functions.h"
#include <stdlib.h> 


/// Create filter shell
ADTF_FILTER_PLUGIN("Emergency Stop", OID_USER_ESTOP, EmergencyStop);


EmergencyStop::EmergencyStop(const tChar* __info):cFilter(__info)
{

	m_bIdsUsStructSet = tFalse;
	m_bIdsSpeedInSet = tFalse;
	m_bIdsSpeedOutSet = tFalse;
	m_szIDBrakeLightSet = tFalse;
	isStopped = tFalse;
	overtakingPossible = tFalse;
	m_szIDsOutputObstacleSet = tFalse;
	m_szIDsInputStopBooleanSet = tFalse;
	m_szIDsOvertakingPossibleBooleanSet = tFalse;
	m_szIDsInputPositionSet = tFalse;
	m_szIDsOutputObstacleFoundToBrainSet = tFalse;
	current_X = 0;
	current_Y = 0;
	current_H = 0;
	m_ObstacleTransmitted = tFalse;
	obstacleCounter = 0;
	m_szIdUsCurrentIndex = 0;
	frontValue = 0.0f;
	m_szIDSteeringResetSet = tFalse;
	resetSteeringSent = tTrue;
	current_steering = 0.0f;
	m_MinUsValue = 400.0f;
	m_szIDsInputSteeringSet = tFalse;

	for(unsigned int i = 0; i < 10; ++i){
		m_UsActivated[i] = tFalse;
	}

	SetPropertyBool("DEBUG::DebutOutput", tFalse);
	SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool("DEBUG::NormalizeSpeedForDebugging", tFalse);
	SetPropertyStr("DEBUG::NormalizeSpeedForDebugging" NSSUBPROP_DESCRIPTION, "Normalize Speed to lie between -1.5 and 1.5?");
	SetPropertyBool("DEBUG::NormalizeSpeedForDebugging" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyFloat("STOP::CurveAngle", 45.0f);
	SetPropertyStr("STOP::CurveAngle" NSSUBPROP_DESCRIPTION, "Minimum Ultrasonic Distance for Full Stop");
	SetPropertyBool("STOP::CurveAngle" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("OVERTAKING::FrontLeftInnerUSThreshold", 100);
	SetPropertyStr("OVERTAKING::FrontLeftInnerUSThreshold" NSSUBPROP_DESCRIPTION, "Amount of cm we allow for objects to be near the inner us sensor before overtaking objects.");
	SetPropertyBool("OVERTAKING::FrontLeftInnerUSThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("OVERTAKING::FrontLeftOuterUSThreshold", 100);
	SetPropertyStr("OVERTAKING::FrontLeftOuterUSThreshold" NSSUBPROP_DESCRIPTION, "Amount of cm we allow for objects to be near the outer us sensor before overtaking objects.");
	SetPropertyBool("OVERTAKING::FrontLeftOuterUSThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("QUEUE::QueueSize", 2);
	SetPropertyStr("QUEUE::QueueSize" NSSUBPROP_DESCRIPTION, "Length of the Ultra sonic queue.");
	SetPropertyBool("QUEUE::QueueSize" NSSUBPROP_ISCHANGEABLE, tTrue);

	// Ertmaliges Befüllen der Queue
	usQueues.resize(10);

	for(int j = 0; j < (int) usQueues.size(); ++j) {
		for(int i = 0; i < m_filterProperties.QueueSize; ++i) {
			usQueues[j].push_back(400.0f);	
		}
	}



	SetPropertyFloat("STOP::StopDistance", 30.0f);
	SetPropertyStr("STOP::StopDistance" NSSUBPROP_DESCRIPTION, "Minimum Ultrasonic Distance for Full Stop");
	SetPropertyBool("STOP::StopDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("STOP::BrakeDistance", 100.0f);
	SetPropertyStr("STOP::BrakeDistance" NSSUBPROP_DESCRIPTION, "Minimum Ultrasonic Distance for Braking");
	SetPropertyBool("STOP::BrakeDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SCALE::FrontCenter", 1.0f);
	SetPropertyStr("SCALE::FrontCenter" NSSUBPROP_DESCRIPTION, "Scaling value for front center ultra sonic sensor");
	SetPropertyBool("SCALE::FrontCenter" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SCALE::FrontInner", 0.80f);
	SetPropertyStr("SCALE::FrontInner" NSSUBPROP_DESCRIPTION, "Scaling value for front inner ultra sonic sensors");
	SetPropertyBool("SCALE::FrontInner" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SCALE::FrontOuter", 0.70f);
	SetPropertyStr("SCALE::FrontOuter" NSSUBPROP_DESCRIPTION, "Scaling value for front outer ultra sonic sensors");
	SetPropertyBool("SCALE::FrontOuter" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SCALE::Side", 0.40f);
	SetPropertyStr("SCALE::Side" NSSUBPROP_DESCRIPTION, "Scaling value for side ultra sonic sensors");
	SetPropertyBool("SCALE::Side" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SCALE::BackOuter", 0.70f);
	SetPropertyStr("SCALE::BackOuter" NSSUBPROP_DESCRIPTION, "Scaling value for back outer ultra sonic sensors");
	SetPropertyBool("SCALE::BackOuter" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SCALE::BackCenter", 0.90f);
	SetPropertyStr("SCALE::BackCenter" NSSUBPROP_DESCRIPTION, "Scaling value for back center ultra sonic sensors");
	SetPropertyBool("SCALE::BackCenter" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("DISTANCESCALING::BaseValue", 0.8f);
	SetPropertyStr("DISTANCESCALING::BaseValue" NSSUBPROP_DESCRIPTION, "Minimum multiplicator when scaling break- and stop-distance.");
	SetPropertyBool("DISTANCESCALING::BaseValue" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("DISTANCESCALING::MaxValue", 2.0f);
	SetPropertyStr("DISTANCESCALING::MaxValue" NSSUBPROP_DESCRIPTION, "Maximum multiplicator when scaling break- and stop-distance.");
	SetPropertyBool("DISTANCESCALING::MaxValue" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyInt("COUNTER::Ceiling", 1000);
	SetPropertyStr("COUNTER::Ceiling" NSSUBPROP_DESCRIPTION, "Ceiling for Counter");
	SetPropertyBool("COUNTER::Ceiling" NSSUBPROP_ISCHANGEABLE, tTrue);
}

EmergencyStop::~EmergencyStop()
{

}

tResult EmergencyStop::Init(tInitStage eStage, __exception)
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
					//für UsStruct
					tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
					RETURN_IF_POINTER_NULL(strDescUsStruct);
					cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0,0,0,"tUltrasonicStruct",strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
					//MediaTypbeschreibungen

					tChar const * strDescObsStruct = pDescManager->GetMediaDescription("tObstacle");
					RETURN_IF_POINTER_NULL(strDescObsStruct);
					cObjectPtr<IMediaType> pTypeObsStruct = new cMediaType(0,0,0,"tObstacle",strDescObsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

					tChar const * strDescPosStruct = pDescManager->GetMediaDescription("tPosition");
					RETURN_IF_POINTER_NULL(strDescPosStruct);
					cObjectPtr<IMediaType> pTypePosStruct = new cMediaType(0,0,0,"tPosition",strDescPosStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

					RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
					RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
					RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean));
					RETURN_IF_FAILED(pTypeObsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionObstacleStruct));
					RETURN_IF_FAILED(pTypePosStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPosition));

					//Pins Erstellen und Registrieren
					RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct",pTypeUsStruct,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));

					RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedControllerIn",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

					RETURN_IF_FAILED(m_oInputSteeringController.Create("SteeringControllerIn",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oInputSteeringController));

					RETURN_IF_FAILED(m_oInputStopBoolean.Create("StopBoolIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oInputStopBoolean));

					RETURN_IF_FAILED(m_oOvertakingPossibleBoolean.Create("OvertakingPossibleIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oOvertakingPossibleBoolean));

					RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedControllerOut",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

					RETURN_IF_FAILED(m_oOutputSteeringReset.Create("SteeringResetOut",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oOutputSteeringReset));

					RETURN_IF_FAILED(m_oOutputObstacle.Create("ObstacleOut",pTypeObsStruct,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oOutputObstacle));

					RETURN_IF_FAILED(m_oOutputObstacleFoundToBrain.Create("ObstacleFoundToBrain",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oOutputObstacleFoundToBrain));

					RETURN_IF_FAILED(m_oOutputBrakeLight.Create("BrakeLightOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));

					RETURN_IF_FAILED(m_oInputPosition.Create("PositionInput",pTypePosStruct,static_cast<IPinEventSink*>(this)));
					RETURN_IF_FAILED(RegisterPin(&m_oInputPosition));


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

tResult EmergencyStop::Shutdown(tInitStage eStage, __exception)
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

tResult EmergencyStop::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		if (pSource == &m_oInputSpeedController)
		{
			ProcessSpeedController(pMediaSample);
		}
		else if (pSource == &m_oInputUsStruct)
		{
			ProcessMinimumValueUs(pMediaSample, desiredSpeed, current_steering);
		}
		else if (pSource == &m_oInputPosition)
		{
			ProcessPosition(pMediaSample);
		}
		else if (pSource == &m_oInputStopBoolean)
		{
			ProcessStopBoolean(pMediaSample);
		}
		else if (pSource == &m_oOvertakingPossibleBoolean)
		{
			ProcessOvertakingPossibleBoolean(pMediaSample);
		}

		else if (pSource == &m_oInputSteeringController)
		{
			ProcessSteering(pMediaSample);
		}

	}

	RETURN_NOERROR;
}

tFloat32 EmergencyStop::NormalizeScalingValue(tFloat32 scalingValue) 
{
	if(scalingValue < 0.001) {
		return 0.0001;
	}

	if(scalingValue > 1.0) {
		return 1.0;
	}

	return scalingValue;
}

tResult EmergencyStop::PropertyChanged(const tChar* strName)
{

	RETURN_IF_FAILED(cFilter::PropertyChanged(strName)); 

	//associate the properties to the member
	if (cString::IsEqual(strName, "STOP::StopDistance"))
		m_filterProperties.stopDist = GetPropertyFloat("STOP::StopDistance");
	else if (cString::IsEqual(strName, "DEBUG::DebutOutput"))
		m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");
	else if (cString::IsEqual(strName, "STOP::BrakeDistance"))
		m_filterProperties.brakeDist = GetPropertyFloat("STOP::BrakeDistance");
	else if (cString::IsEqual(strName, "SCALE::FrontCenter")) {
		m_filterProperties.usScalerFrontCenter = NormalizeScalingValue(GetPropertyFloat("SCALE::FrontCenter"));
	}                                            
	else if (cString::IsEqual(strName, "SCALE::FrontInner")){
		m_filterProperties.usScalerFrontInner = NormalizeScalingValue(GetPropertyFloat("SCALE::FrontInner"));
	}
	else if (cString::IsEqual(strName, "SCALE::FrontOuter")){
		m_filterProperties.usScalerFrontOuter = NormalizeScalingValue(GetPropertyFloat("SCALE::FrontOuter"));
	}
	else if (cString::IsEqual(strName, "SCALE::Side")){
		m_filterProperties.usScalerSide = NormalizeScalingValue(GetPropertyFloat("SCALE::Side"));
	}
	else if (cString::IsEqual(strName, "SCALE::BackOuter")){
		m_filterProperties.usScalerBackOuter = NormalizeScalingValue(GetPropertyFloat("SCALE::BackOuter"));
	}
	else if (cString::IsEqual(strName, "SCALE::BackCenter")){
		m_filterProperties.usScalerBackCenter = NormalizeScalingValue(GetPropertyFloat("SCALE::BackCenter"));
	}
	else if (cString::IsEqual(strName, "COUNTER::Ceiling"))
		m_filterProperties.CounterCeiling = GetPropertyInt("COUNTER::Ceiling");
	else if (cString::IsEqual(strName, "STOP::CurveAngle"))
		m_filterProperties.curveAngle = GetPropertyFloat("STOP::CurveAngle");
	else if (cString::IsEqual(strName, "OVERTAKING::FrontLeftInnerUSThreshold"))
		m_filterProperties.OvertakingFrontLeftInnerThreshold = GetPropertyFloat("OVERTAKING::FrontLeftInnerUSThreshold");
	else if (cString::IsEqual(strName, "OVERTAKING::FrontLeftOuterUSThreshold"))
		m_filterProperties.OvertakingFrontLeftOuterThreshold = GetPropertyFloat("OVERTAKING::FrontLeftOuterUSThreshold");
	else if (cString::IsEqual(strName, "DISTANCESCALING::BaseValue"))
		m_filterProperties.scaleBaseValue = GetPropertyFloat("DISTANCESCALING::BaseValue");
	else if (cString::IsEqual(strName, "DISTANCESCALING::MaxValue"))
		m_filterProperties.scaleMaxValue = GetPropertyFloat("DISTANCESCALING::MaxValue");
	else if (cString::IsEqual(strName, "DEBUG::NormalizeSpeedForDebugging"))
		m_filterProperties.normalizeSpeedForDebugging = GetPropertyBool("DEBUG::NormalizeSpeedForDebugging");
	else if (cString::IsEqual(strName, "QUEUE::QueueSize")) {

		int oldQueueSize = m_filterProperties.QueueSize;
		m_filterProperties.QueueSize = GetPropertyFloat("QUEUE::QueueSize");	
		int newQueueSize =  m_filterProperties.QueueSize;
		int queueSizeDifference =  abs(oldQueueSize - newQueueSize);

		// Ändern der Queueqröße
		for(int j = 0; j < (int) usQueues.size(); ++j) {
			for(int i = 0; i < queueSizeDifference; ++i) {
				if(newQueueSize > oldQueueSize) {
					usQueues[j].push_back(400.0f);	
				} else {
					usQueues[j].pop_front();	
				}
			}
		}
	}
	RETURN_NOERROR;
}

tResult EmergencyStop::ProcessSpeedController(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritSpeedControllerIn);
	tFloat32 speed;
	tUInt32 timestamp;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

		if(!m_bIdsSpeedInSet)
		{
			pCoderInput->GetID("f32Value",m_szIdSpeedInValue);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDInputSpeedControllerTss);
			m_bIdsSpeedInSet = tTrue;
		}

		pCoderInput->Get(m_szIdSpeedInValue, (tVoid*)&speed);
		pCoderInput->Get(m_szIDInputSpeedControllerTss, (tVoid*)&timestamp);
	}

	// Normalize the speed to sensible values so we can debug
	if(m_filterProperties.normalizeSpeedForDebugging) {
		if(fabs(speed) > 15) {
			speed = speed > 0 ? 1.5 : -1.5;
		} else {
			speed /= 10.0;
		}
	}

	// Save the desired speed in a variable
	desiredSpeed = speed;

	{
		// PRINT1("Scaled Distance 0: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, 0.0f));
		// PRINT1("Scaled Distance 0.8: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, 0.8f));
		// PRINT1("Scaled Distance 1.3: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, 1.3f));
		// PRINT1("Scaled Distance 1.5: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, 1.5f));
		// PRINT1("Scaled Stop Distance: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.stopDist, speed));		
		// PRINT1("Scaled Brake Distance: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, speed));		

		tFloat32 scaledStopDistance = ScaleDistanceAccordingToSpeed(m_filterProperties.stopDist, speed);
		tFloat32 scaledBrakeDistance =  ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, speed);

		// PRINT1("MinUSValue: %f", m_MinUsValue);
		//PRINT1("Speed: %f",  speed);
		// PRINT1("Scaled Brake Distance: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, speed));
		// PRINT1("Scaled Stop Distance: %f", ScaleDistanceAccordingToSpeed(m_filterProperties.stopDist, speed));


		// PRINT1("Front inner %f ", GetQueueMedian(usQueues[FRONT_INNER_LEFT]));
		// PRINT1("Front outer %f ", GetQueueMedian(usQueues[FRONT_OUTER_LEFT]));

		PRINT1("Overtaking possible? %d ", overtakingPossible);

		if(m_MinUsValue < scaledStopDistance || isStopped)
		{
			PRINT1("IsStopped: %i", isStopped );

			TransmitBrakeLight(tTrue,GetTime());
			TransmitSpeed(0.0f,timestamp);

			if(!m_ObstacleTransmitted && !isStopped)
			{
				TransmitObstacle(current_X, current_Y, current_H);
				m_ObstacleTransmitted = tTrue;
			}

			tFloat32 frontLeftInnerValue = GetQueueMedian(usQueues[FRONT_INNER_LEFT]);
			tFloat32 frontLeftOuterValue = GetQueueMedian(usQueues[FRONT_OUTER_LEFT]);
			tFloat32 rearCenterValue = GetQueueMedian(usQueues[REAR_CENTER]);

			// PRINT2("frontLeftInnerValue: %f, frontLeftOuterValue: %f",frontLeftInnerValue, frontLeftOuterValue );
			// PRINT2("frontLeftInnerValue: %f, frontLeftOuterValue: %f",frontLeftInnerValue, frontLeftOuterValue );

			if(overtakingPossible && obstacleCounter++ > m_filterProperties.CounterCeiling && !isStopped)
				// && rearCenterValue > scaledStopDistance)
			{
				PRINT("Start overtaking object!");
				// PRINT1("Inner left ultra sonic sensor has distance: %f", frontLeftInnerValue);
				// PRINT1("Outer left ultra sonic sensor has distance: %f", frontLeftOuterValue);
				TransmitObstacleToBrain(tTrue);
			}
			resetSteeringSent = tFalse;
		}
		else if (m_MinUsValue < ScaleDistanceAccordingToSpeed(m_filterProperties.brakeDist, speed))
		{
			PRINT("Breaking.");
			TransmitBrakeLight(tTrue,GetTime());

			speed *= max(0.0f, (m_MinUsValue - scaledStopDistance)/(scaledBrakeDistance - scaledStopDistance));

			if (fabs(current_steering) > 15){
				speed = speed > 0 ? min(speed, 0.7f) : max(speed, -0.7f);
				TransmitBrakeLight(tTrue,GetTime());
			}
			else {
				TransmitBrakeLight(tFalse,GetTime());
			}
			TransmitSpeed(speed,timestamp);
		}
		else
		{
			//PRINT("Driving.");
			TransmitBrakeLight(tFalse,GetTime());

			obstacleCounter = 0;
			m_ObstacleTransmitted = tFalse;

			if(!resetSteeringSent){
				resetSteeringSent = tTrue;
				TransmitSteeringReset(0.0f);
			}
			if (fabs(current_steering) > 15){
				speed = speed > 0 ? min(speed, 0.7f) : max(speed, -0.7f);
			}

			TransmitSpeed(speed,timestamp);
		}

	}
	RETURN_NOERROR;
}

tResult EmergencyStop::ProcessMinimumValueUs(IMediaSample* pMediaSample, tFloat32 speed, tFloat32 steering)
{
	__synchronized_obj(m_critSecMinimumUsValue);
	// tFloat32 &currentMinUsValue = m_oMinimumUsValue.f32Value;

		ActivateUsSensors(speed, steering);
		tFloat32 currentMinUsValue = 400;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionUsStruct, pMediaSample, pCoderInput);
		if(!m_bIdsUsStructSet){

			tBufferID idValue;
			m_szIdUsStructValues.clear();

			pCoderInput->GetID("tFrontCenter.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);
			pCoderInput->GetID("tFrontCenterRight.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tFrontRight.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tSideRight.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tRearRight.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tRearCenter.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tRearLeft.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tSideLeft.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tFrontLeft.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			pCoderInput->GetID("tFrontCenterLeft.f32Value",idValue);
			m_szIdUsStructValues.push_back(idValue);

			m_bIdsUsStructSet = tTrue;
		}

		tFloat32 UsSignalValue;


		for(int i = 0; i < 10; ++i) {
			pCoderInput->Get(m_szIdUsStructValues[i], (tVoid*)&UsSignalValue);

			// Einen Wert aus der Queue nehmen und neuen Wert reinschreiben.
			usQueues[i].pop_front();
			usQueues[i].push_back(UsSignalValue);

			if(!m_UsActivated[i]) {
				continue;
			}

			// Jetzt holen wir uns den aktuellen Median der Queue und verarbeiten ihn dann weiter.
			UsSignalValue = GetQueueMedian(usQueues[i]);

			// Diesen skalieren wir abhängig von der Geschwindigkeit und der Wichtigkeit des Sensors.

			switch(i) {
			case FRONT_CENTER:
				UsSignalValue /= m_filterProperties.usScalerFrontCenter;
				break;
			case FRONT_INNER_RIGHT:
			case FRONT_INNER_LEFT:
				UsSignalValue /= m_filterProperties.usScalerFrontInner;
				break;
			case FRONT_OUTER_RIGHT:
			case FRONT_OUTER_LEFT:
				UsSignalValue /= m_filterProperties.usScalerFrontOuter;
				break;
			case SIDE_RIGHT:
			case SIDE_LEFT:
				UsSignalValue /= m_filterProperties.usScalerSide;
				break;
			case REAR_OUTER_RIGHT:
			case REAR_OUTER_LEFT:
				UsSignalValue /= m_filterProperties.usScalerBackOuter;
				break;
			case REAR_CENTER:
				UsSignalValue /= m_filterProperties.usScalerBackCenter;
				break;
			}

			if(UsSignalValue < currentMinUsValue){
				currentMinUsValue = UsSignalValue;
			}
		}

		m_MinUsValue = currentMinUsValue;

	RETURN_NOERROR;
}

// Skaliert einen Abstandswert anhand eines Geschwindigkeitswertes. Momentan wird folgende Skalierung vorgenommen:
tFloat32  EmergencyStop::ScaleDistanceAccordingToSpeed(tFloat32 distance, tFloat32 speed) 
{
	tFloat32 baseMultiplier = m_filterProperties.scaleBaseValue;
	tFloat32 maxMultiplier = m_filterProperties.scaleMaxValue;

	tFloat32 distanceMultiplier;

	tFloat32 mediumSpeed = 1.3;
	tFloat32 maxSpeed = 1.5;

	// First take the absolute value of the speed value
	speed =  fabs(speed);

	// Then we scale the distance according to the current speed. This will be done by two linear interpolations:
	// For 0 < speed <= mediumSpeed we perform a linear interpolation between zero and baseMultiplier.
	// For speed > mediumSpeed we will interpolate between one and maxMultiplier.

	if(speed <= 0) {
		distanceMultiplier = baseMultiplier;
	} else if(speed > 0 && speed <= mediumSpeed) {
		distanceMultiplier = ((1 - baseMultiplier) / mediumSpeed) * speed + baseMultiplier;
	} else if(speed > mediumSpeed) {
		tFloat32 m = (maxMultiplier - 1) / (maxSpeed - mediumSpeed);
		tFloat32 b = 1 - mediumSpeed * m;
		distanceMultiplier = m * speed + b;
	} else {
		distanceMultiplier = baseMultiplier;
	}

	return distanceMultiplier * distance;
}



tInt EmergencyStop::Sgn(tFloat32 nmbr)
{
	if(nmbr == 0){
		return 0;
	}else if(nmbr > 0)
	{
		return 1;
	}
	else{
		return -1;
	}
}

tResult EmergencyStop::ActivateUsSensors(tFloat32 speed, tFloat32 steering){


	tBool on = tTrue;
	tBool off = tFalse;

	// Erstmal deaktivieren wir defaultmäßig alle Ultraschallsensoren.
	for(int i = 0; i < 10; ++i) {
		m_UsActivated[i] = off;
	}

	tBool leftTurn = steering < -m_filterProperties.curveAngle;
	tBool straight = -m_filterProperties.curveAngle < steering && steering < m_filterProperties.curveAngle;

	// Dann schalten wir die an, die uns im jeweiligen Fall interessieren.
	switch(Sgn(speed)){
	//going forwards
	case 1:
		// PRINT("Forward");
		if(leftTurn){
			// PRINT("Turning left.");
			m_UsActivated[FRONT_INNER_LEFT] = on;
			m_UsActivated[FRONT_CENTER] = on;
		}else if(straight){
			// PRINT("Going straight.");
			m_UsActivated[FRONT_CENTER] = on;
			m_UsActivated[FRONT_INNER_LEFT] = on;
			m_UsActivated[FRONT_INNER_RIGHT] = on;
		} else {
			// PRINT("Turning right.");
			m_UsActivated[FRONT_CENTER] = on;
			m_UsActivated[FRONT_INNER_RIGHT] = on;
		}
		break;
		//Going backwards
	case -1 :
		// PRINT("Backwards");
		if(leftTurn){
			// m_UsActivated[REAR_CENTER] = on;
			// m_UsActivated[REAR_OUTER_LEFT] = on;
		}else if(straight){
			// m_UsActivated[REAR_CENTER] = on;
		} else {
			// m_UsActivated[REAR_OUTER_RIGHT] = on;
			// m_UsActivated[REAR_CENTER] = on;
		}
		break;
		// Wir stehen
	case 0:
		// PRINT("STOPPING");
		m_UsActivated[FRONT_INNER_LEFT] = on;
		m_UsActivated[FRONT_CENTER] = on;
		m_UsActivated[FRONT_INNER_RIGHT] = on;
		break;
	}

	RETURN_NOERROR;
}



tResult EmergencyStop::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
	// PRINT1("E-Stop Speed Out: %f", speed);

	__synchronized_obj(CritSpeedOut);
	//cout << "SpeedValue: " << speed << endl;
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderOutput);

		if (!m_bIdsSpeedOutSet)
		{
			pCoderOutput->GetID("f32Value",m_szIDOutputSpeedControllerValue);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDOutputSpeedControllerTimeStamp);
			m_bIdsSpeedOutSet = tTrue;
		}
		pCoderOutput->Set(m_szIDOutputSpeedControllerValue, (tVoid*)&speed);
		pCoderOutput->Set(m_szIDOutputSpeedControllerTimeStamp, (tVoid*)&timestamp);


		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	m_oOutputSpeedController.Transmit(pMediaSample);

	RETURN_NOERROR;
}

tResult EmergencyStop::ProcessStopBoolean(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritStopBoolIn);
	{

		tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

		if(!m_szIDsInputStopBooleanSet)
		{
			pCoderInput->GetID("bValue",m_szIdStopBoolean);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDInputStopBooleanTS);
			m_szIDsInputStopBooleanSet = tTrue;
		}

		pCoderInput->Get(m_szIdStopBoolean, (tVoid*)&isStopped);
		pCoderInput->Get(m_szIDInputStopBooleanTS, (tVoid*)&timestamp);
	}
	RETURN_NOERROR;
}

tResult EmergencyStop::ProcessOvertakingPossibleBoolean(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritOvertakingIn);
	{

		tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

		if(!m_szIDsOvertakingPossibleBooleanSet)
		{
			pCoderInput->GetID("bValue",m_szIdOvertakingPossibleBoolean);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIDOvertakingPossibleBooleanTS);
			m_szIDsOvertakingPossibleBooleanSet = tTrue;
		}

		pCoderInput->Get(m_szIdOvertakingPossibleBoolean, (tVoid*)&overtakingPossible);
		pCoderInput->Get(m_szIDOvertakingPossibleBooleanTS, (tVoid*)&timestamp);
	}
	RETURN_NOERROR;
}



tResult EmergencyStop::TransmitSteeringReset(tFloat32 offsetValue)
{
	__synchronized_obj(CritSteeringResetOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderOutput);

		if (!m_szIDSteeringResetSet)
		{
			pCoderOutput->GetID("f32Value",m_szIdSteeringResetValue);
			m_szIDSteeringResetSet = tTrue;
		}
		pCoderOutput->Set(m_szIdSteeringResetValue, (tVoid*)&offsetValue);


		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputSteeringReset.Transmit(pMediaSample));

	RETURN_NOERROR;
}

tResult EmergencyStop::TransmitBrakeLight(tBool finished, tUInt32 timestamp)
{
	__synchronized_obj(CritBrakeLightOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);
		tTimeStamp ts = GetTime();
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


tResult EmergencyStop::TransmitObstacleToBrain(tBool obstacle)
{
	__synchronized_obj(CritObstacleToBrainOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		tTimeStamp ts = GetTime();
		__adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

		if (!m_szIDsOutputObstacleFoundToBrainSet)
		{
			pCoderOutput->GetID("bValue",m_szIDOutputObstacleFoundToBrainID);
			pCoderOutput->GetID("ui32ArduinoTimestamp",m_szIDOutputObstacleFoundToBrainIDTs);
			m_szIDsOutputObstacleFoundToBrainSet = tTrue;
		}
		pCoderOutput->Set(m_szIDOutputObstacleFoundToBrainID, (tVoid*)&obstacle);
		pCoderOutput->Set(m_szIDOutputObstacleFoundToBrainIDTs, (tVoid*)&ts);


		pMediaSample->SetTime(_clock->GetStreamTime());
	}
	RETURN_IF_FAILED(m_oOutputObstacleFoundToBrain.Transmit(pMediaSample));

	RETURN_NOERROR;
}




tResult EmergencyStop::ProcessSteering(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritSteeringIn);
	{
		// tTimeStamp timestamp;
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

		if(!m_szIDsInputSteeringSet)
		{
			pCoderInput->GetID("f32Value",m_szIdSteeringInValue);
			pCoderInput->GetID("ui32ArduinoTimestamp",m_szIdSteeringInValueTS);
			m_szIDsInputSteeringSet = tTrue;
		}
		pCoderInput->Get(m_szIdSteeringInValue, (tVoid*)&current_steering);
	}
	RETURN_NOERROR;
}

tResult EmergencyStop::ProcessPosition(IMediaSample* pMediaSample)
{
	__synchronized_obj(CritPositionIn);
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionPosition, pMediaSample, pCoderInput);

		if(!m_szIDsInputPositionSet)
		{
			pCoderInput->GetID("f32x",m_szPositionXValue);
			pCoderInput->GetID("f32y",m_szPositionYValue);
			pCoderInput->GetID("f32heading",m_szPositionHValue);
			m_szIDsInputPositionSet = tTrue;
		}

		pCoderInput->Get(m_szPositionXValue, (tVoid*)&current_X);
		pCoderInput->Get(m_szPositionYValue, (tVoid*)&current_Y);
		pCoderInput->Get(m_szPositionHValue, (tVoid*)&current_H);

	}
	//TransmitDistance(m_filterProperties.distanceToStopLine, GetTime());
	RETURN_NOERROR;
}

tResult EmergencyStop::TransmitObstacle(tFloat32 xPos, tFloat32 yPos, tFloat32 hPos)
{
	__synchronized_obj(CritObstacleOut);
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionObstacleStruct->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pDescriptionObstacleStruct, pMediaSample, pCoderOutput);

		if (!m_szIDsOutputObstacleSet)
		{
			pCoderOutput->GetID("f32x",m_szIDOutputObstacleXValue);
			pCoderOutput->GetID("f32y",m_szIDOutputObstacleYValue);
			m_szIDsOutputObstacleSet = tTrue;
		}
		tFloat32 xPosObs =  xPos + 0.4 * cos(hPos);
		tFloat32 yPosObs =  yPos +  0.4 * sin(hPos);
		pCoderOutput->Set(m_szIDOutputObstacleXValue, (tVoid*)&xPosObs);
		pCoderOutput->Set(m_szIDOutputObstacleYValue, (tVoid*)&yPosObs);

		PRINT2("Object transmitted: %f, %f",xPosObs, yPosObs);

	}

	//pMediaSample->SetTime(_clock->GetStreamTime());
	RETURN_IF_FAILED(m_oOutputObstacle.Transmit(pMediaSample));


	RETURN_NOERROR;
}

tUInt32 EmergencyStop::GetTime()
{
	return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 EmergencyStop::Betrag(tFloat32 input)
{
	tFloat32 Betrag = (input >= 0) ? input : -input;
	return Betrag;
}

