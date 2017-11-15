#include <math.h>
#include "stdafx.h"
#include "BaseControl.h"
#include <unistd.h>
#include <stdexcept>
#include <iostream>
#include "functions.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("Base Control", OID_USER_BASECONTROL, BaseControl);

BaseControl::BaseControl(const tChar* __info) :
    cFilter(__info) {
	SetPropertyBool("DEBUG::DebugOutput", tFalse);
	SetPropertyStr("DEBUG::DebugOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?");
	SetPropertyBool("DEBUG::DebugOutput" NSSUBPROP_ISCHANGEABLE, tTrue);
    

	m_szIDsOutputParkingspaceSet = tFalse;
	m_szIDsInputPositionSet = tFalse;
	m_szIDsOutputBrakeLightSet = tFalse;
	m_szIDResetLaneSteeringSet = tFalse;
	m_bIdsSpeedInSet = tFalse;
	m_bIdsSpeedOutSet = tFalse;
	m_szIDsOutputSteeringSet = tFalse;
	m_szIDsInputDistanceSet = tFalse;
	m_szIDsInputStopBooleanSet = tFalse;
	m_szIDsInputSteeringSet = tFalse;
	m_szIDsCarStateInSet = tFalse;
	m_szIDsFinishedSet = tFalse;
    m_szIDsOutputBlinkerLinksSet = tFalse;
    m_szIDsOutputBlinkerRechtsSet = tFalse;
    m_szIDsOutputRuecklichtSet = tFalse;
	m_szIDsOutputDriveDistanceSet = tFalse;
	m_szIDResetDriveDistanceSet = tFalse;
	m_szIDsOutputFinishedSet = tFalse;
	m_InputUsSet = tFalse;
	m_DistanceToLineSet = tFalse;
	m_szIDsInputParkingIDSet = tFalse;
	m_SwitchLaneBufferSet = tFalse;
	currentParkingID = 0;
	currentParkingspaceIndex = 0;
	CurrentCarstate = 0;
	isStopped = tFalse;
	TurnInProcess = tFalse;
	OverallDistanceSet = tFalse;
	calibCounter = 0;
	DistanceAtStartOfTurn = 0;
	CurrentCarstate = 0;
	LastCarstate = 0;
	DistanceTravelled = 0;
	ManeuverPhase = 0;
	StopDistance = 0.0f;
	current_X = 0.0f;
	current_Y = 0.0f;
	current_H = 0;
	parkingSpacesXMLLoaded = tFalse;
	parkingSpacesXMLInputValid = tFalse;
	LaneKeepingActive = tFalse;
	parkingCounter = 0;

	lastParkingspace.ParkingspaceID = -1;
	lastParkingspace.ParkingspaceX = -1000.0f;
	lastParkingspace.ParkingspaceY = -1000.0f;
	lastParkingspace.ParkingspaceStatus = tFalse;
	lastParkingspace.ParkingspaceDirection = -1000.0f;
	lastParkingspace.Valid = tFalse;

	DEG2RAD = (M_PI / 180.0);

	SetPropertyStr("Configuration", "roadSign.xml");
	SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
	SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");

	SetPropertyFloat("SPEEDS::defaultSpeed", -10.0f);
	SetPropertyStr("SPEEDS::defaultSpeed" NSSUBPROP_DESCRIPTION, "Default Speed used");
	SetPropertyBool("SPEEDS::defaultSpeed" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SPEEDS::turnLeftSpeed", -15.0f);
	SetPropertyStr("SPEEDS::turnLeftSpeed" NSSUBPROP_DESCRIPTION, "Speed used in a left turn");
	SetPropertyBool("SPEEDS::turnLeftSpeed" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("SPEEDS::turnRightSpeed", -20.0f);
	// TODO soll das so? turnLeftSpeed
	SetPropertyStr("SPEEDS::turnLeftSpeed" NSSUBPROP_DESCRIPTION, "Speed used in a right turn");
	SetPropertyBool("SPEEDS::turnLeftSpeed" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::RightDistance", 1);
	SetPropertyStr("TURN::RightDistance" NSSUBPROP_DESCRIPTION, "Distance of right turn");
	SetPropertyBool("TURN::RightDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::RightDistanceTwo", 1);
	SetPropertyStr("TURN::RightDistanceTwo" NSSUBPROP_DESCRIPTION, "Distance of right turn");
	SetPropertyBool("TURN::RightDistanceTwo" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::LeftDistanceOne", 1);
	SetPropertyStr("TURN::LeftDistanceOne" NSSUBPROP_DESCRIPTION, "Distance of left turn");
	SetPropertyBool("TURN::LeftDistanceOne" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::LeftDistanceTwo", 1);
	SetPropertyStr("TURN::LeftDistanceTwo" NSSUBPROP_DESCRIPTION, "Distance of left turn");
	SetPropertyBool("TURN::LeftDistanceTwo" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::LeftSteeringOne", -30.0);
	SetPropertyStr("TURN::LeftSteeringOne" NSSUBPROP_DESCRIPTION, "Distance of left turn");
	SetPropertyBool("TURN::LeftSteeringOne" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::LeftSteeringTwo", -30.0);
	SetPropertyStr("TURN::LeftSteeringTwo" NSSUBPROP_DESCRIPTION, "Distance of left turn");
	SetPropertyBool("TURN::LeftSteeringTwo" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::RightROISwitch", 0.0);
	SetPropertyStr("TURN::RightROISwitch" NSSUBPROP_DESCRIPTION, "Distance of left turn");
	SetPropertyBool("TURN::RightROISwitch" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::RightSteering", 75.0);
	SetPropertyStr("TURN::RightSteering" NSSUBPROP_DESCRIPTION, "Distance of left turn");
	SetPropertyBool("TURN::RightSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURN::RightSteeringTwo", 75.0);
	SetPropertyStr("TURN::RightSteeringTwo" NSSUBPROP_DESCRIPTION, "Distance of left turn");
	SetPropertyBool("TURN::RightSteeringTwo" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("STOP::DistanceToStopline", 0.1);
	SetPropertyStr("STOP::DistanceToStopline" NSSUBPROP_DESCRIPTION, "Distance to drive after stopline detected");
	SetPropertyBool("STOP::DistanceToStopline" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("STOP::DistanceToStoplineRight", 0.1);
	SetPropertyStr("STOP::DistanceToStoplineRight" NSSUBPROP_DESCRIPTION, "Distance to drive after stopline detected");
	SetPropertyBool("STOP::DistanceToStoplineRight" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("STOP::DistanceToStoplineLeft", 0.1);
	SetPropertyStr("STOP::DistanceToStoplineLeft" NSSUBPROP_DESCRIPTION, "Distance to drive after stopline detected");
	SetPropertyBool("STOP::DistanceToStoplineLeft" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("STEERING::ZeroCalibration", 0);
	SetPropertyStr("STEERING::ZeroCalibration" NSSUBPROP_DESCRIPTION, "Value to be added to every steering output");
	SetPropertyBool("STEERING::ZeroCalibration" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("TURNOUTRIGHT::PhaseTwoSteering", 75.0);
	SetPropertyStr("TURNOUTRIGHT::PhaseTwoSteering" NSSUBPROP_DESCRIPTION, "SteeringValue during second phase of turnout maneuver");
	SetPropertyBool("TURNOUTRIGHT::PhaseTwoSteering" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTRIGHT::PhaseThreeSteering", -75.0);
	SetPropertyStr("TURNOUTRIGHT::PhaseThreeSteering" NSSUBPROP_DESCRIPTION, "SteeringValue during third phase of turnout maneuver");
	SetPropertyBool("TURNOUTRIGHT::PhaseThreeSteering" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTRIGHT::PhaseOneDistance", 0.4f);
	SetPropertyStr("TURNOUTRIGHT::PhaseOneDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of turnout maneuver");
	SetPropertyBool("TURNOUTRIGHT::PhaseOneDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTRIGHT::PhaseTwoDistance", 0.6f);
	SetPropertyStr("TURNOUTRIGHT::PhaseTwoDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of turnout maneuver");
	SetPropertyBool("TURNOUTRIGHT::PhaseTwoDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTRIGHT::PhaseThreeDistance", -0.6f);
	SetPropertyStr("TURNOUTRIGHT::PhaseThreeDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of turnout maneuver");
	SetPropertyBool("TURNOUTRIGHT::PhaseThreeDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTRIGHT::PhaseFourDistance", -0.2f);
	SetPropertyStr("TURNOUTRIGHT::PhaseFourDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of turnout maneuver");
	SetPropertyBool("TURNOUTRIGHT::PhaseFourDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTLEFT::PhaseTwoSteering", -75.0);
	SetPropertyStr("TURNOUTLEFT::PhaseTwoSteering" NSSUBPROP_DESCRIPTION, "SteeringValue during second phase of turnout maneuver");
	SetPropertyBool("TURNOUTLEFT::PhaseTwoSteering" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTLEFT::PhaseThreeSteering", 0.0);
	SetPropertyStr("TURNOUTLEFT::PhaseThreeSteering" NSSUBPROP_DESCRIPTION, "SteeringValue during third phase of turnout maneuver");
	SetPropertyBool("TURNOUTLEFT::PhaseThreeSteering" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTLEFT::PhaseOneDistance", 0.4f);
	SetPropertyStr("TURNOUTLEFT::PhaseOneDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of turnout maneuver");
	SetPropertyBool("TURNOUTLEFT::PhaseOneDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTLEFT::PhaseTwoDistance", 0.6f);
	SetPropertyStr("TURNOUTLEFT::PhaseTwoDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of turnout maneuver");
	SetPropertyBool("TURNOUTLEFT::PhaseTwoDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("TURNOUTLEFT::PhaseThreeDistance", -0.6f);
	SetPropertyStr("TURNOUTLEFT::PhaseThreeDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of turnout maneuver");
	SetPropertyBool("TURNOUTLEFT::PhaseThreeDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("PARKING::PhaseOneDistance", -0.18f);
	SetPropertyStr("PARKING::PhaseOneDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseOneDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::PhaseTwoDistance", 0.65f);
	SetPropertyStr("PARKING::PhaseTwoDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseTwoDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::PhaseThreeDistance", -0.45f);
	SetPropertyStr("PARKING::PhaseThreeDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseThreeDistance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

	SetPropertyFloat("PARKING::PhaseFourDistance", -0.5f);
	SetPropertyStr("PARKING::PhaseFourDistance" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseFourDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::PhaseOneSteering", 10.0f);
	SetPropertyStr("PARKING::PhaseOneSteering" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseOneSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::PhaseTwoSteering", -100.0f);
	SetPropertyStr("PARKING::PhaseTwoSteering" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseTwoSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::PhaseThreeSteering", 80.0f);
	SetPropertyStr("PARKING::PhaseThreeSteering" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseThreeSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::PhaseFourSteering", 0.0f);
	SetPropertyStr("PARKING::PhaseFourSteering" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::PhaseFourSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::DistanceToParkingspace", 0.3f);
	SetPropertyStr("PARKING::DistanceToParkingspace" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::DistanceToParkingspace" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PARKING::DistanceBetweenParkingspaces", 0.5f);
	SetPropertyStr("PARKING::DistanceBetweenParkingspaces" NSSUBPROP_DESCRIPTION, "Distance during first phase of Parking maneuver");
	SetPropertyBool("PARKING::DistanceBetweenParkingspaces" NSSUBPROP_ISCHANGEABLE,	tTrue);

	SetPropertyFloat("OVERTAKE::ZeroDist", -0.0f);
	SetPropertyStr("OVERTAKE::ZeroDist" NSSUBPROP_DESCRIPTION, "Distance Zero");
	SetPropertyBool("OVERTAKE::ZeroDist" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::OneDist", -0.0f);
	SetPropertyStr("OVERTAKE::OneDist" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::OneDist" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::TwoDist", -0.0f);
	SetPropertyStr("OVERTAKE::TwoDist" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::TwoDist" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::ThreeDist", -0.0f);
	SetPropertyStr("OVERTAKE::ThreeDist" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::ThreeDist" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::FourDist", -0.0f);
	SetPropertyStr("OVERTAKE::FourDist" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::FourDist" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::OneSteering", -0.0f);
	SetPropertyStr("OVERTAKE::OneSteering" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::OneSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::TwoSteering", -0.0f);
	SetPropertyStr("OVERTAKE::TwoSteering" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::TwoSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::ThreeSteering", -0.0f);
	SetPropertyStr("OVERTAKE::ThreeSteering" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::ThreeSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::FourSteering", -0.0f);
	SetPropertyStr("OVERTAKE::FourSteering" NSSUBPROP_DESCRIPTION, "Distance One");
	SetPropertyBool("OVERTAKE::FourSteering" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("OVERTAKE::Distance", -0.0f);
	SetPropertyStr("OVERTAKE::Distance" NSSUBPROP_DESCRIPTION, "Distance");
	SetPropertyBool("OVERTAKE::Distance" NSSUBPROP_ISCHANGEABLE, tTrue);
}

BaseControl::~BaseControl() {

}

tResult BaseControl::Init(tInitStage eStage, __exception)
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
            tChar const * strDescInt32SignalValue = pDescManager->GetMediaDescription("tInt32SignalValue");
            RETURN_IF_POINTER_NULL(strDescInt32SignalValue);
            cObjectPtr<IMediaType> pTypeInt32SignalValue = new cMediaType(0, 0, 0, "tInt32SignalValue", strDescInt32SignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
            //media type für bools
            tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
            RETURN_IF_POINTER_NULL(strDescBool);
            cObjectPtr<IMediaType> pTypeSignalBool = new cMediaType(0,0,0,"tBoolSignalValue",strDescBool, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
            //für DriverStruct
            tChar const * strDescDrivStr = pDescManager->GetMediaDescription("tDriverStruct");
            RETURN_IF_POINTER_NULL(strDescDrivStr);
            cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tDriverStruct", strDescDrivStr, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
            tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
            RETURN_IF_POINTER_NULL(strDescUsStruct);
            cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0,0,0,"tUltrasonicStruct",strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

            tChar const * strDescPosStruct = pDescManager->GetMediaDescription("tPosition");
            RETURN_IF_POINTER_NULL(strDescPosStruct);
            cObjectPtr<IMediaType> pTypePosStruct = new cMediaType(0,0,0,"tPosition",strDescPosStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

            tChar const * strDescParkStruct = pDescManager->GetMediaDescription("tParkingSpace");
            RETURN_IF_POINTER_NULL(strDescParkStruct);
            cObjectPtr<IMediaType> pTypeParkStruct = new cMediaType(0,0,0,"tParkingSpace",strDescParkStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

            tChar const * strDescDriveDistance = pDescManager->GetMediaDescription("driveDistanceCommand");
            RETURN_IF_POINTER_NULL(strDescDriveDistance);
            cObjectPtr<IMediaType> pTypeDriveDistance = new cMediaType(0,0,0,"driveDistanceCommand",strDescDriveDistance, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

            //MediaTypbeschreibungen
            RETURN_IF_FAILED(pTypeParkStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionParking));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue2));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue3));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue4));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue5));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue6));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue7));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue8));
            RETURN_IF_FAILED(pTypeInt32SignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValueInt))
                RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean));
            RETURN_IF_FAILED(pTypeSignalBool->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolean2));
            RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescDriverStruct));
            RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
            RETURN_IF_FAILED(pTypePosStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPosition));
            RETURN_IF_FAILED(pTypeDriveDistance->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDriveDistance));

            //Pins Erstellen und Registrieren
            RETURN_IF_FAILED(m_DistanceToLine.Create("DistanceToLine",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_DistanceToLine));

            RETURN_IF_FAILED(m_oInputPosition.Create("PositionInput",pTypePosStruct,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputPosition));

            RETURN_IF_FAILED(m_oOutputParkingspace.Create("ParkingOut",pTypeParkStruct,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputParkingspace));

            RETURN_IF_FAILED(m_oInputLaneSteering.Create("LaneSteeringIn",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputLaneSteering));

            RETURN_IF_FAILED(m_oInputCarState.Create("CarStateIn",pTypeInt32SignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputCarState));

            RETURN_IF_FAILED(m_oInputParkingID.Create("ParkingIDIn",pTypeInt32SignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputParkingID));

            RETURN_IF_FAILED(m_DriverStructInputPin.Create("Driver_Struct", pType2, this));
            RETURN_IF_FAILED(RegisterPin(&m_DriverStructInputPin));

            RETURN_IF_FAILED(m_oInputStopBoolean.Create("StopBoolIn",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputStopBoolean));

            RETURN_IF_FAILED(m_oInputFinished.Create("ManoeuverFinished",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputFinished));
            
            RETURN_IF_FAILED(m_oOutputBlinkerLinks.Create("BlinkerLinksOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputBlinkerLinks));

            RETURN_IF_FAILED(m_oOutputBlinkerRechts.Create("BlinkerRechtsOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputBlinkerRechts));

            RETURN_IF_FAILED(m_oOutputRuecklicht.Create("RuecklichtOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputRuecklicht));

            RETURN_IF_FAILED(m_oOutputDriveDistance.Create("DriveDistanceOut",pTypeDriveDistance,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputDriveDistance));

            RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedControllerOut",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

            RETURN_IF_FAILED(m_oOutputSteering.Create("SteeringOut",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

            RETURN_IF_FAILED(m_oOutputDebug.Create("DebugOut",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputDebug));

            RETURN_IF_FAILED(m_oInputDistanceSinceLastSample.Create("DistanceIn",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputDistanceSinceLastSample));

            RETURN_IF_FAILED(m_oOutputResetDriveDistance.Create("ResetDriveDistance",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputResetDriveDistance));

            RETURN_IF_FAILED(m_oOutputFinished.Create("ManoeuverFinishedOut",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputFinished));

            RETURN_IF_FAILED(m_oOutputBrakeLight.Create("BrakeLightOutput",pTypeSignalBool,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));

            RETURN_IF_FAILED(m_oOutputResetLaneSteering.Create("ResetLaneSteering",pTypeSignalValue,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputResetLaneSteering));

            RETURN_IF_FAILED(m_InputUsStruct.Create("UltraSonicIn",pTypeUsStruct,static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_InputUsStruct));

            RETURN_IF_FAILED(m_SwitchLaneOut.Create("SwitchLaneOut", pTypeSignalBool, static_cast<IPinEventSink*>(this)));
            RETURN_IF_FAILED(RegisterPin(&m_SwitchLaneOut));
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

tResult BaseControl::Shutdown(tInitStage eStage, __exception)
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

tResult BaseControl::LoadConfiguration() {
	cFilename fileConfig = GetPropertyStr("Configuration");

	if (fileConfig.IsEmpty()) {
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	// create absolute path for marker configuration file
	ADTF_GET_CONFIG_FILENAME(fileConfig);
	fileConfig = fileConfig.CreateAbsolutePath(".");

	if (cFileSystem::Exists(fileConfig)) {
		cDOM oDOM;
		oDOM.Load(fileConfig);
		cDOMElementRefList oElems;

		if (IS_OK(oDOM.FindNodes("configuration/parkingSpace", oElems))) {

			int currentSetIndex = 0;

			for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem) {
				ParkingspaceStruct item;
				item.ParkingspaceID = tUInt16((*itElem)->GetAttribute("id", "0").AsInt32());
				item.ParkingspaceX = tFloat32((*itElem)->GetAttribute("x", "0").AsFloat64());
				item.ParkingspaceY = tFloat32((*itElem)->GetAttribute("y", "0").AsFloat64());
				item.ParkingspaceStatus = tFloat32((*itElem)->GetAttribute("status", "0").AsBool());
				item.ParkingspaceDirection = tFloat32((*itElem)->GetAttribute("direction", "0").AsFloat64());
				item.ParkingspaceDirection *= DEG2RAD; // convert to radians
				item.Valid = tTrue;

				PRINT5("Load Parking Space: Id %d XY %f %f Status %i Direction %f", item.ParkingspaceID, item.ParkingspaceX, item.ParkingspaceY, item.ParkingspaceStatus, item.ParkingspaceDirection);

				// Wenn wir beim ersten Parkplatz sind, erzeugen wir ein neues Set und fügen den Parkplatz ein.
				if (!lastParkingspace.Valid) {
					set<ParkingspaceStruct> firstSet;
					firstSet.insert(firstSet.begin(), item);
					parkingSpaceList.push_back(firstSet);

				} else {
					// Wenn der neue Parkplatz in einem Meter Umkreis zum letzen ist, fügen wir ihn zu letzten Gruppe hinzu.
					if (Distance(item, lastParkingspace) < 1.0) {
						parkingSpaceList[currentSetIndex].insert(item);
					} else {
						// Ansonsten erzeugen wir ein neues Set und fügen den aktuellen und die (eventuell) folgenden Parkplätze in dieses ein.
						set<ParkingspaceStruct> nextSet;
						nextSet.insert(item);
						parkingSpaceList.push_back(nextSet);
						currentSetIndex++;
					}
				}
				lastParkingspace = item;

			}
		}
	} else {
		LOG_ERROR("Configuration file does not exist");
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	RETURN_NOERROR;
}

tResult BaseControl::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample) {
	// int setCounter = 1;
    // int elementCounter = 1;
    

    // for (vector<set<ParkingspaceStruct> >::iterator it = parkingSpaceList.begin(); it != parkingSpaceList.end(); ++it) {
    //     PRINT1("Set %i", setCounter++);
        
    //     for (set<ParkingspaceStruct>::iterator setIt = (*it).begin(); setIt != (*it).end(); ++setIt) {

    //         PRINT2("Element %i : ID %i", elementCounter++, ( * setIt).ParkingspaceID);
            
    //     }
    //     elementCounter = 1;
        
    // }
    // for(int i = 1;i <  7;i++) {
    //     PRINT2("%i, Set index: %i", i, GetParkingspaceSetIndex(i));
    // }
    for(int i = 0;i < 4;i++) {
        PRINT2("%i, set at: %i", i + 1, SetAt(parkingSpaceList.at(0),i).ParkingspaceID);
    }
    for(int i = 0;i < 2;i++) {
        PRINT2("%i, set at: %i", i + 5, SetAt(parkingSpaceList.at(1),i).ParkingspaceID);
    }
    



    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {

        if (!parkingSpacesXMLLoaded) {
            // Load the xml for the parking spaces.
            LoadConfiguration();
            parkingSpacesXMLLoaded = tTrue;

            if (parkingSpaceList.size() == 0) {
                LOG_ERROR("Achtung! In der xml-Datei waren keine Parkplätze eingetragen!");
            } else {
                parkingSpacesXMLInputValid = tTrue;
            }
        }

        if (calibCounter <= 100) {
            TransmitSteering(0.0f, GetTime());
            calibCounter++;
        }
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &m_DistanceToLine) {
            ProcessDistanceToLine(pMediaSample);
        } else if (pSource == &m_oInputStopBoolean) {
            ProcessStopBoolean(pMediaSample);
        } else if (pSource == &m_oInputPosition) {
            ProcessPosition(pMediaSample);
        } else if (pSource == &m_oInputFinished) {
            ProcessFinished(pMediaSample);
        } else if (pSource == &m_oInputLaneSteering) {
            if (LaneKeepingActive || CurrentCarstate == 1 || CurrentCarstate == 6 || CurrentCarstate == 12 || CurrentCarstate == 13 || CurrentCarstate == 14
                || (CurrentCarstate == 11 && LastCarstate != 3 && LastCarstate != 4)) {
                ProcessSteering(pMediaSample);
                TransmitSteering(SteeringValue, SteeringTimestamp);
            }
        } else if (pSource == &m_oInputCarState) {

            //TransmitDistance(1.0f,GetTime());
            ProcessCarState(pMediaSample);
            ActOnCarState(CurrentCarstate);
            //cout << "CarState " << CurrentCarstate << " received" << endl;
        } else if (pSource == &m_InputUsStruct) {
            ProcessUS(pMediaSample);
        } else if (pSource == &m_oInputParkingID) {
            ProcessParkingID(pMediaSample);
        }
    }
    RETURN_NOERROR;
}

tResult BaseControl::ProcessPosition(IMediaSample* pMediaSample) {
    __synchronized_obj(m_critSecPositionInput);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionPosition, pMediaSample, pCoderInput);

        if (!m_szIDsInputPositionSet) {
            pCoderInput->GetID("f32x", m_szPositionXValue);
            pCoderInput->GetID("f32y", m_szPositionYValue);
            pCoderInput->GetID("f32heading", m_szPositionHValue);
            m_szIDsInputPositionSet = tTrue;
        }

        pCoderInput->Get(m_szPositionXValue, (tVoid*) &current_X);
        pCoderInput->Get(m_szPositionYValue, (tVoid*) &current_Y);
        pCoderInput->Get(m_szPositionHValue, (tVoid*) &current_H);

    }
    //TransmitDistance(m_filterProperties.distanceToStopLine, GetTime());
    RETURN_NOERROR;
}

tResult BaseControl::ProcessStopBoolean(IMediaSample* pMediaSample) {

    __synchronized_obj(m_critSecStopBool);
    {
        tTimeStamp timestamp;
        __adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderInput);

        if (!m_szIDsInputStopBooleanSet) {
            pCoderInput->GetID("bValue", m_szIdStopBoolean);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDInputStopBooleanTS);
            m_szIDsInputStopBooleanSet = tTrue;
        }

        pCoderInput->Get(m_szIdStopBoolean, (tVoid*) &isStopped);
        pCoderInput->Get(m_szIDInputStopBooleanTS, (tVoid*) &timestamp);
    }
    //TransmitDistance(m_filterProperties.distanceToStopLine, GetTime());
    RETURN_NOERROR;
}

BaseControl::ParkingspaceStruct BaseControl::GetClosestParkingspace(std::vector<set<ParkingspaceStruct> >) {

    ParkingspaceStruct currentParkingspace;
    tFloat32 currentMinimalDistance = std::numeric_limits<float>::max();

    if (parkingSpacesXMLInputValid) {

        for (vector<set<ParkingspaceStruct> >::iterator it = parkingSpaceList.begin(); it != parkingSpaceList.end(); ++it) {

            for (set<ParkingspaceStruct>::iterator setIt = (*it).begin(); setIt != (*it).end(); ++setIt) {

                tFloat32 currentDistance = Distance(current_X, current_Y, setIt->ParkingspaceX, setIt->ParkingspaceY);

                if (currentDistance < currentMinimalDistance) {
                    currentParkingspace = *setIt;
                    currentMinimalDistance = currentDistance;
                }
            }
        }

    } else {
        currentParkingspace.ParkingspaceID = -1;
        currentParkingspace.ParkingspaceX = -1000.0f;
        currentParkingspace.ParkingspaceY = -1000.0f;
        currentParkingspace.ParkingspaceStatus = tFalse;
        currentParkingspace.ParkingspaceDirection = -1000.0f;
        currentParkingspace.Valid = tFalse;
    }

    return currentParkingspace;
}

tFloat32 BaseControl::Distance(tFloat32 x1, tFloat32 y1, tFloat32 x2, tFloat32 y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

tFloat32 BaseControl::Distance(ParkingspaceStruct a, ParkingspaceStruct b) {
    return Distance(a.ParkingspaceX, a.ParkingspaceY, b.ParkingspaceX, b.ParkingspaceY);
}

tResult BaseControl::ProcessUS(IMediaSample* pMediaSample) {
    __synchronized_obj(m_critSecUltrasonicInput);
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionUsStruct, pMediaSample, pCoderInput);
        if (!m_InputUsSet) {
            pCoderInput->GetID("tSideRight.f32Value", m_ParkinSpaceDepthValueID);
            m_InputUsSet = tTrue;
        }
        pCoderInput->Get(m_ParkinSpaceDepthValueID, (tVoid*) &m_ParkinSpaceDepthValue);
    }

    RETURN_NOERROR;
}
tResult BaseControl::ProcessFinished(IMediaSample* pMediaSample) {
    tBool manoeuverStatus;
    tUInt32 timestamp;
    __synchronized_obj(m_critSecFinishedInput);
    {
        // tTimeStamp timestamp;
        __adtf_sample_read_lock_mediadescription(m_pDescriptionBoolean2, pMediaSample, pCoderInput);

        if (!m_szIDsFinishedSet) {
            pCoderInput->GetID("bValue", m_szIdFinished);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDFinishedTS);
            m_szIDsFinishedSet = tTrue;
        }

        pCoderInput->Get(m_szIdFinished, (tVoid*) &manoeuverStatus);
        pCoderInput->Get(m_szIDFinishedTS, (tVoid*) &timestamp);
    }

    TransmitResetDriveDistance(tTrue, GetTime());

    ParkingspaceStruct closestParkingspace;
    tBool parkplatzBelegt;
    tInt32 currentParkingspaceSetIndex;

    
    switch (CurrentCarstate) {
    case 0:
        break;
    case 3:
        switch (ManeuverPhase) {
        case 1:
            TransmitDistance(m_filterProperties.turnRightDistTwo, tFalse);
            TransmitSteering(m_filterProperties.turnRightSteeringTwo, GetTime());
            ManeuverPhase++;
            //cout << "Phase 2 reached" << endl;
            break;
        default:
            //cout << "Phase 3 reached" << endl;
            TransmitSteering(m_filterProperties.steeringZeroCalibration, GetTime());
            TransmitFinished(tTrue);
            ManeuverPhase = 1;
            break;
        }
        break;
    case 4:
        switch (ManeuverPhase) {
        case 1:
            TransmitDistance(m_filterProperties.turnLeftDistTwo, tFalse);
            TransmitSteering(m_filterProperties.turnLeftSteeringTwo, GetTime());
            ManeuverPhase++;
            //cout << "Phase 2 reached" << endl;
            break;
        default:
            //cout << "Phase 3 reached" << endl;
            TransmitFinished(tTrue);
            ManeuverPhase = 1;
            break;
        }
        break;
    case 5:

        switch (ManeuverPhase) {
        case 1:
            PRINT("Park Phase 1");
            TransmitRuecklicht(tFalse);
            //cout << "parking phase two reached" << endl;
            TransmitDistance(m_filterProperties.parkingPhaseTwoDistance, tFalse);
            TransmitSteering(m_filterProperties.parkingPhaseTwoSteering, GetTime());
            ManeuverPhase++;
            break;
        case 2:
            TransmitRuecklicht(tTrue);

            PRINT("Park Phase 2");
            //cout << "parking phase three reached" << endl;
            TransmitDistance(m_filterProperties.parkingPhaseThreeDistance, tFalse);
            TransmitSteering(m_filterProperties.parkingPhaseThreeSteering, GetTime());
            ManeuverPhase++;
            break;
        case 3:
            PRINT("Park Phase 3");
            //cout << "parking phase four reached" << endl;
            TransmitDistance(m_filterProperties.parkingPhaseFourDistance, tFalse);
            TransmitSteering(m_filterProperties.parkingPhaseFourSteering, GetTime());
            ManeuverPhase++;
            break;
        default:
            PRINT("Fertig geparkt!");
            TransmitRuecklicht(tFalse);
            TransmitFinished(tTrue);
            ManeuverPhase = 1;
            break;
        }
        break;
        // Turn out right
    case 9:

        //TransmitResetDriveDistance(tTrue,GetTime());
        PRINT("TurnOutRight Switch reached");
        switch (ManeuverPhase) {
        case (tInt) 1:
            TransmitDistance(m_filterProperties.turnOutRightPhaseTwoDistance, tFalse);
            TransmitSteering(m_filterProperties.turnOutRightPhaseTwoSteering, GetTime());
            ManeuverPhase++;
            PRINT("TurnOutRight Phase 2");

            //cout << "Phase 2 reached" << endl;
            break;
        case (tInt) 2:

            TransmitRuecklicht(tTrue);
            
            TransmitDistance(m_filterProperties.turnOutRightPhaseThreeDistance, tFalse);
            TransmitSteering(m_filterProperties.turnOutRightPhaseThreeSteering, GetTime());
            ManeuverPhase = 3;
            PRINT("TurnOutRight Phase 3");

            //cout << "Phase 3 reached" << endl;
            break;
        case (tInt) 3:
            TransmitDistance(m_filterProperties.turnOutRightPhaseFourDistance, tFalse);
            TransmitSteering(m_filterProperties.steeringZeroCalibration, GetTime());
            ManeuverPhase = 4;
            PRINT("TurnOutRight Phase 4");

            //cout << "Phase 4 reached" << endl;
            break;
        case (tInt) 4:
            //cout << "Phase 5 reached" << endl;
            PRINT("TurnOutRightFinished reached");
            
            TransmitRuecklicht(tFalse);

            //  Ans Backend weitergeben, dass der Parkplatz frei ist.
            if(currentParkingID == 0) {
                closestParkingspace = GetClosestParkingspace(parkingSpaceList);
            } else {
                closestParkingspace = GetClosestParkingspaceByID(currentParkingID);
            }
            if(closestParkingspace.ParkingspaceID != -1) {
                TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 0, CurrentCarstate);
            }
			


            TransmitFinished(tTrue);
            ManeuverPhase = 1;
            break;
        }
        break;
        // Turn out left
    case 10:
		

        switch (ManeuverPhase) {
        case 1:
            //	TransmitResetDriveDistance(tTrue,GetTime());
            TransmitDistance(m_filterProperties.turnOutLeftPhaseTwoDistance,
                             tFalse);
            TransmitSteering(m_filterProperties.turnOutLeftPhaseTwoSteering, GetTime());
            ManeuverPhase++;
            PRINT("TurnOutLeft Phase 1");
            break;
        case 2:
            //TransmitResetDriveDistance(tTrue,GetTime());
            TransmitRuecklicht(tTrue);
            TransmitDistance(m_filterProperties.turnOutLeftPhaseThreeDistance,
                             tFalse);
            TransmitSteering(m_filterProperties.turnOutLeftPhaseThreeSteering, GetTime());
            PRINT("TurnOutLeft Phase 2");
            
            ManeuverPhase++;
            break;
        case 3:
            PRINT("TurnOutLeftFinished");
            TransmitRuecklicht(tFalse);

            //  Ans Backend weitergeben, dass der Parkplatz frei ist.
            if(currentParkingID == 0) {
                closestParkingspace = GetClosestParkingspace(parkingSpaceList);
            } else {
                closestParkingspace = GetClosestParkingspaceByID(currentParkingID);
            }
						
            if(closestParkingspace.ParkingspaceID != -1) {
                TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 0, CurrentCarstate);
            }

            TransmitFinished(tTrue);
            ManeuverPhase = 1;
            break;
        }
        break;
    case 11:
        TransmitBrakeLight(tFalse);
        CurrentCarstate = 1;
        ActOnCarState(CurrentCarstate);
        break;

    case 13:
        
        // Wir reseten den Counter für die Parkplätze
        parkingCounter = 0;

        // Zuerst suchen wir den aktuellen Parkplatz aus der Liste. Falls das nicht funktioniert, suchen wir den nächsten zu aktuellen Position.
        currentParkingspaceSetIndex = GetParkingspaceSetIndex(currentParkingID);
        // PRINT1("Set index: %i", currentParkingspaceSetIndex);

        closestParkingspace = SetAt(parkingSpaceList.at(currentParkingspaceSetIndex), parkingCounter);
        // PRINT1("closest parking space: %i", closestParkingspace.ParkingspaceID);

        parkplatzBelegt = m_ParkinSpaceDepthValue < 50;
        // PRINT1("US Daten: %f", m_ParkinSpaceDepthValue);

        

        if (parkplatzBelegt) {
            PRINT("Parkplatz war belegt.");

            // Wenn der Parkplatz belegt ist, melden wir dies ans Backend und fahren zum nächsten Parkplatz.
            CurrentCarstate = 14;
            //Hier wird der falsche Parkplatz gesendet. Es wird der "Jury" Parkplatz gesendet und nicht der vor dem wir gerade stehen.
			
            if(closestParkingspace.ParkingspaceID != -1) {
                TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 1, CurrentCarstate);
            }	

            ActOnCarState(CurrentCarstate);

        } else {
            if (parkingCounter < currentParkingspaceIndex) {
                PRINT2("Parkplatz hat zu kleine ID. %i < %i", parkingCounter, currentParkingspaceIndex);

                // Ist der Parkplatz frei, aber die ID falsch, fahren wir weiter und melden einen freien Parkplatz ans Backend.
                CurrentCarstate = 14;

                //Hier wird der falsche Parkplatz gesendet. Es wird der "Jury" Parkplatz gesendet und nicht der vor dem wir gerade stehen.
				
                if(closestParkingspace.ParkingspaceID != -1) {
                    TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 0, CurrentCarstate);
                }	

                ActOnCarState(CurrentCarstate);

            } else {
                // Ist der Parkplatz frei und die ID richtig, parken wir ein.
                PRINT1("Ich stehe vor Parkplatz Nr.: %i und parke ein.", parkingCounter);
                parkingCounter = 0;
                CurrentCarstate = 5;

                if(closestParkingspace.ParkingspaceID != -1) {
                    TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 1, CurrentCarstate);
                }	
                ActOnCarState(CurrentCarstate);
            }
        }
        break;
    case 14:
        parkingCounter++;
        
        currentParkingspaceSetIndex = GetParkingspaceSetIndex(currentParkingID);
        closestParkingspace = SetAt(parkingSpaceList.at(currentParkingspaceSetIndex), parkingCounter);
        

        parkplatzBelegt = m_ParkinSpaceDepthValue < 50;
        PRINT1("US Daten: %f", m_ParkinSpaceDepthValue);
        if (parkplatzBelegt) {
            //Hier wird der falsche Parkplatz gesendet. Es wird der "Jury" Parkplatz gesendet und nicht der vor dem wir gerade stehen.
			
            if(closestParkingspace.ParkingspaceID != -1) {
                TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 1, CurrentCarstate);
            }	


            ActOnCarState(CurrentCarstate);
        } else {
            if (parkingCounter < currentParkingspaceIndex) {
                PRINT2("Parkplatz hat zu kleine ID. %i < %i", parkingCounter, currentParkingspaceIndex);
                // Ist der Parkplatz frei, aber die ID falsch, fahren wir weiter und melden einen freien Parkplatz ans Backend.
                CurrentCarstate = 14;
                //Hier wird der falsche Parkplatz gesendet. Es wird der "Jury" Parkplatz gesendet und nicht der vor dem wir gerade stehen.
				
                if(closestParkingspace.ParkingspaceID != -1) {
                    TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 0, CurrentCarstate);
                }	

                ActOnCarState(CurrentCarstate);
            } else {
                // Ist der Parkplatz frei und die ID richtig, parken wir ein.
                PRINT1("Ich stehe vor Parkplatz Nr.: %i und parke ein.", parkingCounter);
                parkingCounter = 0;
                CurrentCarstate = 5;
				
                if(closestParkingspace.ParkingspaceID != -1) {
                    TransmitParking(closestParkingspace.ParkingspaceID, closestParkingspace.ParkingspaceX, closestParkingspace.ParkingspaceY, 1, CurrentCarstate);
                }	
                ActOnCarState(CurrentCarstate);
            }
        }
        break;
        // Überholen
    case 15:
        switch (ManeuverPhase) {
        case 1:
            TransmitRuecklicht(tFalse);
            TransmitBlinkerLinks(tTrue);
            
            TransmitResetLaneSteering(0.0f, GetTime());
            LaneKeepingActive = tTrue;
            TransmitSwitchLane(tFalse);
            TransmitDistance(1.5f + m_filterProperties.OvertakeDist, tFalse);
            ManeuverPhase++;
            break;
        case 2:
            TransmitBlinkerLinks(tFalse);
            TransmitBlinkerRechts(tTrue);

            TransmitDistance(1.5f, tFalse);
            TransmitSwitchLane(tTrue);
            ManeuverPhase++;

            break;
        case 3:
            TransmitBlinkerRechts(tFalse);

            TransmitFinished(tTrue);
            CurrentCarstate = 1;
            ActOnCarState(CurrentCarstate);
            //ManeuverPhase++;

            break;
        }
        break;
    default:
        TransmitFinished(tTrue);

    }

    RETURN_NOERROR;
}

BaseControl::ParkingspaceStruct BaseControl::SetAt(set<ParkingspaceStruct> s, int index) 
{
    int i = 0;
    ParkingspaceStruct newParkingspace;
    newParkingspace.ParkingspaceID = -1;
    newParkingspace.ParkingspaceX = 0.0f;
    newParkingspace.ParkingspaceY = 0.0f;
    newParkingspace.ParkingspaceStatus = tFalse;
    newParkingspace.ParkingspaceDirection = 90.0f;
    newParkingspace.Valid = tTrue;



    for (std::set<ParkingspaceStruct>::reverse_iterator setIt = s.rbegin(); setIt != s.rend(); ++setIt) {
        if(i++ == index) {
            newParkingspace.ParkingspaceID = (*setIt).ParkingspaceID  ;
            newParkingspace.ParkingspaceX = (*setIt).ParkingspaceX;
            newParkingspace.ParkingspaceY = (*setIt).ParkingspaceY;
            newParkingspace.ParkingspaceStatus = (*setIt).ParkingspaceStatus;
            newParkingspace.ParkingspaceDirection = (*setIt).ParkingspaceDirection;
            newParkingspace.Valid = (*setIt).Valid;
            return newParkingspace;
        }
    }

    
    return newParkingspace;
}




tResult BaseControl::PropertyChanged(const tChar* strName) {
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    if (cString::IsEqual(strName, "DEBUG::DebugOutput")) {
        m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebugOutput");
    }

    //associate the properties to the member
    if (cString::IsEqual(strName, "SPEEDS::defaultSpeed"))
        m_filterProperties.defaultSpeed = GetPropertyFloat("SPEEDS::defaultSpeed");
    else if (cString::IsEqual(strName, "SPEEDS::turnRightSpeed"))
        m_filterProperties.turnRightSpeed = GetPropertyFloat("SPEEDS::turnRightSpeed");
    else if (cString::IsEqual(strName, "SPEEDS::turnLeftSpeed"))
        m_filterProperties.turnLeftSpeed = GetPropertyFloat("SPEEDS::turnLeftSpeed");
    else if (cString::IsEqual(strName, "TURN::RightDistance"))
        m_filterProperties.turnRightDist = GetPropertyFloat("TURN::RightDistance");
    else if (cString::IsEqual(strName, "TURN::RightDistanceTwo"))
        m_filterProperties.turnRightDistTwo = GetPropertyFloat("TURN::RightDistanceTwo");
    else if (cString::IsEqual(strName, "TURN::LeftDistanceOne"))
        m_filterProperties.turnLeftDistOne = GetPropertyFloat("TURN::LeftDistanceOne");
    else if (cString::IsEqual(strName, "TURN::LeftSteeringOne"))
        m_filterProperties.turnLeftSteeringOne = GetPropertyFloat("TURN::LeftSteeringOne");
    else if (cString::IsEqual(strName, "TURN::LeftDistanceTwo"))
        m_filterProperties.turnLeftDistTwo = GetPropertyFloat("TURN::LeftDistanceTwo");
    else if (cString::IsEqual(strName, "TURN::LeftSteeringTwo"))
        m_filterProperties.turnLeftSteeringTwo = GetPropertyFloat("TURN::LeftSteeringTwo");
    else if (cString::IsEqual(strName, "TURN::RightSteering"))
        m_filterProperties.turnRightSteering = GetPropertyFloat("TURN::RightSteering");
    else if (cString::IsEqual(strName, "TURN::RightSteeringTwo"))
        m_filterProperties.turnRightSteeringTwo = GetPropertyFloat("TURN::RightSteeringTwo");
    else if (cString::IsEqual(strName, "TURN::RightROISwitch"))
        m_filterProperties.turnRightROISwitch = GetPropertyFloat("TURN::RightROISwitch");
    else if (cString::IsEqual(strName, "STOP::DistanceToStopline"))
        m_filterProperties.distanceToStopLine = GetPropertyFloat("STOP::DistanceToStopline");
    else if (cString::IsEqual(strName, "STOP::DistanceToStoplineRight"))
        m_filterProperties.distanceToStopLineRight = GetPropertyFloat("STOP::DistanceToStoplineRight");
    else if (cString::IsEqual(strName, "STOP::DistanceToStoplineLeft"))
        m_filterProperties.distanceToStopLineLeft = GetPropertyFloat("STOP::DistanceToStoplineLeft");
    else if (cString::IsEqual(strName, "STEERING::ZeroCalibration"))
        m_filterProperties.steeringZeroCalibration = GetPropertyFloat("STEERING::ZeroCalibration");
    else if (cString::IsEqual(strName, "TURNOUTRIGHT::PhaseTwoSteering"))
        m_filterProperties.turnOutRightPhaseTwoSteering = GetPropertyFloat("TURNOUTRIGHT::PhaseTwoSteering");
    else if (cString::IsEqual(strName, "TURNOUTRIGHT::PhaseThreeSteering"))
        m_filterProperties.turnOutRightPhaseThreeSteering = GetPropertyFloat("TURNOUTRIGHT::PhaseThreeSteering");
    else if (cString::IsEqual(strName, "TURNOUTRIGHT::PhaseOneDistance"))
        m_filterProperties.turnOutRightPhaseOneDistance = GetPropertyFloat("TURNOUTRIGHT::PhaseOneDistance");
    else if (cString::IsEqual(strName, "TURNOUTRIGHT::PhaseTwoDistance"))
        m_filterProperties.turnOutRightPhaseTwoDistance = GetPropertyFloat("TURNOUTRIGHT::PhaseTwoDistance");
    else if (cString::IsEqual(strName, "TURNOUTRIGHT::PhaseThreeDistance"))
        m_filterProperties.turnOutRightPhaseThreeDistance = GetPropertyFloat("TURNOUTRIGHT::PhaseThreeDistance");
    else if (cString::IsEqual(strName, "TURNOUTRIGHT::PhaseFourDistance"))
        m_filterProperties.turnOutRightPhaseFourDistance = GetPropertyFloat("TURNOUTRIGHT::PhaseFourDistance");
    else if (cString::IsEqual(strName, "TURNOUTLEFT::PhaseTwoSteering"))
        m_filterProperties.turnOutLeftPhaseTwoSteering = GetPropertyFloat("TURNOUTLEFT::PhaseTwoSteering");
    else if (cString::IsEqual(strName, "TURNOUTLEFT::PhaseThreeSteering"))
        m_filterProperties.turnOutLeftPhaseThreeSteering = GetPropertyFloat("TURNOUTLEFT::PhaseThreeSteering");
    else if (cString::IsEqual(strName, "TURNOUTLEFT::PhaseOneDistance"))
        m_filterProperties.turnOutLeftPhaseOneDistance = GetPropertyFloat("TURNOUTLEFT::PhaseOneDistance");
    else if (cString::IsEqual(strName, "TURNOUTLEFT::PhaseTwoDistance"))
        m_filterProperties.turnOutLeftPhaseTwoDistance = GetPropertyFloat("TURNOUTLEFT::PhaseTwoDistance");
    else if (cString::IsEqual(strName, "TURNOUTLEFT::PhaseThreeDistance"))
        m_filterProperties.turnOutLeftPhaseThreeDistance = GetPropertyFloat("TURNOUTLEFT::PhaseThreeDistance");
    else if (cString::IsEqual(strName, "PARKING::PhaseOneDistance"))
        m_filterProperties.parkingPhaseOneDistance = GetPropertyFloat("PARKING::PhaseOneDistance");
    else if (cString::IsEqual(strName, "PARKING::PhaseTwoDistance"))
        m_filterProperties.parkingPhaseTwoDistance = GetPropertyFloat("PARKING::PhaseTwoDistance");
    else if (cString::IsEqual(strName, "PARKING::PhaseThreeDistance"))
        m_filterProperties.parkingPhaseThreeDistance = GetPropertyFloat("PARKING::PhaseThreeDistance");
    else if (cString::IsEqual(strName, "PARKING::PhaseFourDistance"))
        m_filterProperties.parkingPhaseFourDistance = GetPropertyFloat("PARKING::PhaseFourDistance");
    else if (cString::IsEqual(strName, "PARKING::PhaseOneSteering"))
        m_filterProperties.parkingPhaseOneSteering = GetPropertyFloat("PARKING::PhaseOneSteering");
    else if (cString::IsEqual(strName, "PARKING::PhaseTwoSteering"))
        m_filterProperties.parkingPhaseTwoSteering = GetPropertyFloat("PARKING::PhaseTwoSteering");
    else if (cString::IsEqual(strName, "PARKING::PhaseThreeSteering"))
        m_filterProperties.parkingPhaseThreeSteering = GetPropertyFloat("PARKING::PhaseThreeSteering");
    else if (cString::IsEqual(strName, "PARKING::PhaseFourSteering"))
        m_filterProperties.parkingPhaseFourSteering = GetPropertyFloat("PARKING::PhaseFourSteering");
    else if (cString::IsEqual(strName, "PARKING::DistanceToParkingspace"))
        m_filterProperties.distanceToParkingspace = GetPropertyFloat("PARKING::DistanceToParkingspace");
    else if (cString::IsEqual(strName, "PARKING::DistanceBetweenParkingspaces"))
        m_filterProperties.distanceBetweenParkingspaces = GetPropertyFloat("PARKING::DistanceBetweenParkingspaces");
    else if (cString::IsEqual(strName, "OVERTAKE::ZeroDist"))
        m_filterProperties.OvertakePhaseZeroDist = GetPropertyFloat("OVERTAKE::ZeroDist");
    else if (cString::IsEqual(strName, "OVERTAKE::OneDist"))
        m_filterProperties.OvertakePhaseOneDist = GetPropertyFloat("OVERTAKE::OneDist");
    else if (cString::IsEqual(strName, "OVERTAKE::TwoDist"))
        m_filterProperties.OvertakePhaseTwoDist = GetPropertyFloat("OVERTAKE::TwoDist");
    else if (cString::IsEqual(strName, "OVERTAKE::ThreeDist"))
        m_filterProperties.OvertakePhaseThreeDist = GetPropertyFloat("OVERTAKE::ThreeDist");
    else if (cString::IsEqual(strName, "OVERTAKE::FourDist"))
        m_filterProperties.OvertakePhaseFourDist = GetPropertyFloat("OVERTAKE::FourDist");
    else if (cString::IsEqual(strName, "OVERTAKE::OneSteering"))
        m_filterProperties.OvertakePhaseOneSteering = GetPropertyFloat("OVERTAKE::OneSteering");
    else if (cString::IsEqual(strName, "OVERTAKE::TwoSteering"))
        m_filterProperties.OvertakePhaseTwoSteering = GetPropertyFloat("OVERTAKE::TwoSteering");
    else if (cString::IsEqual(strName, "OVERTAKE::ThreeSteering"))
        m_filterProperties.OvertakePhaseThreeSteering = GetPropertyFloat("OVERTAKE::ThreeSteering");
    else if (cString::IsEqual(strName, "OVERTAKE::FourSteering"))
        m_filterProperties.OvertakePhaseFourSteering = GetPropertyFloat("OVERTAKE::FourSteering");
    else if (cString::IsEqual(strName, "OVERTAKE::Distance"))
        m_filterProperties.OvertakeDist = GetPropertyFloat("OVERTAKE::Distance");
    RETURN_NOERROR;
}

tResult BaseControl::ProcessDistanceToLine(IMediaSample* pMediaSample) {
    __synchronized_obj(m_critSecDistanceToLineInput);
    tUInt32 timestamp;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue2, pMediaSample, pCoderInput);

        if (!m_DistanceToLineSet) {
            pCoderInput->GetID("f32Value", m_DIstanceToLineBuffer);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_DIstanceToLineBufferTs);

            m_DistanceToLineSet = tTrue;
        }
        pCoderInput->Get(m_DIstanceToLineBuffer, (tVoid*) &DistanceToLine);
        pCoderInput->Get(m_DIstanceToLineBufferTs, (tVoid*) &timestamp);

    }
    RETURN_NOERROR;
}

tResult BaseControl::TransmitSpeed(tFloat32 speed, tUInt32 timestamp) {

    __synchronized_obj(CritSpeedOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue3->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue3, pMediaSample, pCoderOutput);

        if (!m_bIdsSpeedOutSet) {
            pCoderOutput->GetID("f32Value", m_szIDOutputSpeedControllerValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputSpeedControllerTimeStamp);
            m_bIdsSpeedOutSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputSpeedControllerValue, (tVoid*) &speed);
        pCoderOutput->Set(m_szIDOutputSpeedControllerTimeStamp, (tVoid*) &timestamp);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputSpeedController.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult BaseControl::TransmitSwitchLane(tBool value) {
    __synchronized_obj(CritSwitchLaneOut);

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));
    tTimeStamp ts = GetTime();
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

        if (!m_SwitchLaneBufferSet) {
            pCoderOutput->GetID("bValue", m_SwitchLaneBuffer);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_SwitchLaneBufferTs);
            m_SwitchLaneBufferSet = tTrue;
        }
        pCoderOutput->Set(m_SwitchLaneBuffer, (tVoid*) &value);
        pCoderOutput->Set(m_SwitchLaneBufferTs, (tVoid*) &ts);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_SwitchLaneOut.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::TransmitDistance(tFloat32 distance, tBool brake) {
    __synchronized_obj(CritDriveDIstanceOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionDriveDistance->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDriveDistance, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputDriveDistanceSet) {
            pCoderOutput->GetID("f32Value", m_szDriveDistanceOutValue);
            pCoderOutput->GetID("bValue", m_szIDOutputDriveDistanceTss);
            m_szIDsOutputDriveDistanceSet = tTrue;
        }
        pCoderOutput->Set(m_szDriveDistanceOutValue, (tVoid*) &distance);
        pCoderOutput->Set(m_szIDOutputDriveDistanceTss, (tVoid*) &brake);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputDriveDistance.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::ProcessSteering(IMediaSample* pMediaSample) {
    __synchronized_obj(m_critSecSteeringInput);
    {
        // tTimeStamp timestamp;
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue4, pMediaSample, pCoderInput);

        if (!m_szIDsInputSteeringSet) {
            pCoderInput->GetID("f32Value", m_szIdSteeringInValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDInputSteeringControllerTs);
            m_szIDsInputSteeringSet = tTrue;
        }
        pCoderInput->Get(m_szIdSteeringInValue, (tVoid*) &SteeringValue);
        pCoderInput->Get(m_szIDInputSteeringControllerTs, (tVoid*) &SteeringTimestamp);
    }
    RETURN_NOERROR;
}

tResult BaseControl::ProcessCarState(IMediaSample* pMediaSample) {
    __synchronized_obj(m_critSecCarStateInput);
    tTimeStamp ts;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValueInt, pMediaSample, pCoderInput);

        if (!m_szIDsCarStateInSet) {
            pCoderInput->GetID("intValue", m_szIdCarStateInValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDCarStateInTs);
            m_szIDsCarStateInSet = tTrue;
        }
        pCoderInput->Get(m_szIdCarStateInValue, (tVoid*) &CurrentCarstate);
        pCoderInput->Get(m_szIDCarStateInTs, (tVoid*) &ts);
    }
    RETURN_NOERROR;
}

tResult BaseControl::ProcessParkingID(IMediaSample* pMediaSample) {
    __synchronized_obj(m_critSecParkingIDInput);
    tTimeStamp ts;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValueInt, pMediaSample, pCoderInput);

        if (!m_szIDsInputParkingIDSet) {
            pCoderInput->GetID("intValue", m_szParkingID);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szParkingIDTs);
            m_szIDsInputParkingIDSet = tTrue;
        }
        pCoderInput->Get(m_szParkingID, (tVoid*) &currentParkingID);
        pCoderInput->Get(m_szParkingIDTs, (tVoid*) &ts);
    }

    // PRINT1("Jury Parkplatz: %i", currentParkingID);

    for (std::vector<set<ParkingspaceStruct> >::iterator it = parkingSpaceList.begin(); it != parkingSpaceList.end(); ++it) {
        int i = 0;
        for (std::set<ParkingspaceStruct>::reverse_iterator setIt = (*it).rbegin(); setIt != (*it).rend(); ++setIt) {
            //PRINT1("(*setIt).ParkingspaceID in Process: %i",(*setIt).ParkingspaceID);
            if ((*setIt).ParkingspaceID == currentParkingID) {
                currentParkingspaceIndex = i;

                // PRINT1("ParkplatzID im Parkplatz: %i", currentParkingspaceIndex);
                RETURN_NOERROR;
            }

            i++;
        }
    }
    RETURN_NOERROR;

}

tInt32 BaseControl::GetParkingspaceSetIndex(tInt32 parkingspaceIndex) {
    
    tInt32 i = 0;
    for (std::vector<set<ParkingspaceStruct> >::iterator it = parkingSpaceList.begin(); it != parkingSpaceList.end(); ++it) {
        
        for (std::set<ParkingspaceStruct>::iterator setIt = (*it).begin(); setIt != (*it).end(); ++setIt) {
            if( (*setIt).ParkingspaceID ==  parkingspaceIndex) {
                return i;
            }
        }
        i++;
    }

    return 0;
}


BaseControl::ParkingspaceStruct BaseControl::GetClosestParkingspaceByID(int currentParkingspaceID) {
    tInt32 setIndex = GetParkingspaceSetIndex(currentParkingspaceID);
        
    ParkingspaceStruct newParkingspace =  { - 1, 0.0f, 0.0f, tFalse, 90.0f, tTrue};
    

    tInt32 i = 0;
    set<ParkingspaceStruct> currentSet = parkingSpaceList.at(setIndex);
        
    for (std::set<ParkingspaceStruct>::reverse_iterator setIt = currentSet.rbegin(); setIt != currentSet.rend(); ++setIt) {
        
        if ((*setIt).ParkingspaceID == currentParkingspaceID) {
            newParkingspace.ParkingspaceID = (*setIt).ParkingspaceID;
            newParkingspace.ParkingspaceX = (*setIt).ParkingspaceX;
            newParkingspace.ParkingspaceY = (*setIt).ParkingspaceY;
            
            return newParkingspace;
        }
    }

    return newParkingspace;
}

tResult BaseControl::TransmitSteering(tFloat32 steering, tUInt32 timestamp) {
    __synchronized_obj(CritSteeringOut);
    //cout << "Steering Value: " << steering << endl;
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue6->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    tFloat32 steeringV = steering + m_filterProperties.steeringZeroCalibration;
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue6, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputSteeringSet) {
            pCoderOutput->GetID("f32Value", m_szIDOutputSteeringValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputSteeringTimeStamp);
            m_szIDsOutputSteeringSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputSteeringValue, (tVoid*) &steeringV);
        pCoderOutput->Set(m_szIDOutputSteeringTimeStamp, (tVoid*) &timestamp);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    m_oOutputSteering.Transmit(pMediaSample);
    RETURN_NOERROR;
}

tResult BaseControl::TransmitDebug(tFloat32 steering, tUInt32 timestamp) {
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue7->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue7, pMediaSample, pCoderOutput);

        if (!m_szIDsDebugSet) {
            pCoderOutput->GetID("f32Value", m_szIdDebugValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDDebugTs);
            m_szIDsDebugSet = tTrue;
        }
        pCoderOutput->Set(m_szIdDebugValue, (tVoid*) &steering);
        pCoderOutput->Set(m_szIDDebugTs, (tVoid*) &timestamp);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputDebug.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::TransmitResetDriveDistance(tBool boolean, tUInt32 timestamp) {
    __synchronized_obj(CritResetDriveDistance);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

        if (!m_szIDResetDriveDistanceSet) {
            pCoderOutput->GetID("bValue", m_szIdResetDriveDistance);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdResetDriveDistanceTs);
            m_szIDResetDriveDistanceSet = tTrue;
        }
        pCoderOutput->Set(m_szIdResetDriveDistance, (tVoid*) &boolean);
        pCoderOutput->Set(m_szIdResetDriveDistanceTs, (tVoid*) &timestamp);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputResetDriveDistance.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::TransmitFinished(tBool value) {
    __synchronized_obj(CritManeuverFinishedOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    tTimeStamp ts = GetTime();
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputFinishedSet) {
            pCoderOutput->GetID("bValue", m_szIDOutputFinishedValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputFinishedTimeStamp);
            m_szIDsOutputFinishedSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputFinishedValue, (tVoid*) &value);
        pCoderOutput->Set(m_szIDOutputFinishedTimeStamp, (tVoid*) &ts);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputFinished.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::TransmitBlinkerLinks(tBool value) {
    __synchronized_obj(CritBlinkerLinksOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    tTimeStamp ts = GetTime();
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputBlinkerLinksSet) {
            pCoderOutput->GetID("bValue", m_szIDOutputBlinkerLinksValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputBlinkerLinksTimeStamp);
            m_szIDsOutputBlinkerLinksSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputBlinkerLinksValue, (tVoid*) &value);
        pCoderOutput->Set(m_szIDOutputBlinkerLinksTimeStamp, (tVoid*) &ts);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputBlinkerLinks.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::TransmitRuecklicht(tBool value) {
    __synchronized_obj(CritRuecklichtOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    tTimeStamp ts = GetTime();
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputRuecklichtSet) {
            pCoderOutput->GetID("bValue", m_szIDOutputRuecklichtValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputRuecklichtTimeStamp);
            m_szIDsOutputRuecklichtSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputRuecklichtValue, (tVoid*) &value);
        pCoderOutput->Set(m_szIDOutputRuecklichtTimeStamp, (tVoid*) &ts);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputRuecklicht.Transmit(pMediaSample));
    RETURN_NOERROR;
}


tResult BaseControl::TransmitBlinkerRechts(tBool value) {
    __synchronized_obj(CritBlinkerRechtsOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    tTimeStamp ts = GetTime();
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputBlinkerRechtsSet) {
            pCoderOutput->GetID("bValue", m_szIDOutputBlinkerRechtsValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputBlinkerRechtsTimeStamp);
            m_szIDsOutputBlinkerRechtsSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputBlinkerRechtsValue, (tVoid*) &value);
        pCoderOutput->Set(m_szIDOutputBlinkerRechtsTimeStamp, (tVoid*) &ts);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputBlinkerRechts.Transmit(pMediaSample));
    RETURN_NOERROR;
}



tResult BaseControl::TransmitResetLaneSteering(tFloat32 dynWidth, tUInt32 timestamp) {
    __synchronized_obj(CritResetLaneSteering);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderOutput);

        if (!m_szIDResetLaneSteeringSet) {
            pCoderOutput->GetID("f32Value", m_szIdResetLaneSteering);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdResetLaneSteeringTs);
            m_szIDResetLaneSteeringSet = tTrue;
        }
        pCoderOutput->Set(m_szIDResetLaneSteeringSet, (tVoid*) &dynWidth);
        pCoderOutput->Set(m_szIdResetLaneSteeringTs, (tVoid*) &timestamp);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputResetLaneSteering.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::TransmitBrakeLight(tBool BoolVal) {
    __synchronized_obj(CritBrakeLightOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBoolean->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    tTimeStamp ts = GetTime();
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBoolean, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputBrakeLightSet) {
            pCoderOutput->GetID("bValue", m_szIDOutputBrakeLightValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDOutputBrakeLightTimeStamp);
            m_szIDsOutputBrakeLightSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputBrakeLightValue, (tVoid*) &BoolVal);
        pCoderOutput->Set(m_szIDOutputBrakeLightTimeStamp, (tVoid*) &ts);

        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputBrakeLight.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult BaseControl::TransmitParking(tInt16 Identifier, tFloat32 xPos, tFloat32 yPos, tUInt16 Status, tUInt32 CurrentCarstate) {
    __synchronized_obj(CritParkingOut);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionParking->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionParking, pMediaSample, pCoderOutput);

        if (!m_szIDsOutputParkingspaceSet) {
            pCoderOutput->GetID("i16Identifier", m_szIDOutputParkingIDValue);
            pCoderOutput->GetID("f32x", m_szIDOutputParkingXValue);
            pCoderOutput->GetID("f32y", m_szIDOutputParkingYValue);
            pCoderOutput->GetID("ui16Status", m_szIDOutputParkingStatusValue);
            m_szIDsOutputParkingspaceSet = tTrue;
        }
        pCoderOutput->Set(m_szIDOutputParkingIDValue, (tVoid*) &Identifier);
        pCoderOutput->Set(m_szIDOutputParkingXValue, (tVoid*) &xPos);
        pCoderOutput->Set(m_szIDOutputParkingYValue, (tVoid*) &yPos);
        pCoderOutput->Set(m_szIDOutputParkingStatusValue, (tVoid*) &Status);
        pMediaSample->SetTime(_clock->GetStreamTime());
    }
    RETURN_IF_FAILED(m_oOutputParkingspace.Transmit(pMediaSample));
    RETURN_NOERROR;
}

//  Reset all lights when returning to normal driving modes.
//  This functions is used in case car states that use lights could not fully execute.
tResult BaseControl::ResetLights() {
    TransmitBrakeLight(tFalse);
    TransmitRuecklicht(tFalse);
    TransmitBlinkerLinks(tFalse);
    TransmitBlinkerRechts(tFalse);
    RETURN_NOERROR;
}

tResult BaseControl::ActOnCarState(tUInt32 carstate) {
    LaneKeepingActive = tFalse;
    ManeuverPhase = 1;
    TransmitResetDriveDistance(tTrue, GetTime());
    usleep(100);
    switch (carstate) {
            // CarState 0: Stoppen

        case 0:
        

            LastCarstate = 0;
            TransmitBrakeLight(tTrue);
            TransmitResetDriveDistance(tTrue, GetTime());
        

            break;
            //  CarState 1: Weiter den Straßenmarkierungen folgen
        case 1:
            TransmitSteering(0.0f, GetTime());
            if (LastCarstate == 9 || LastCarstate == 2 || LastCarstate == 10) {
                TransmitResetLaneSteering(60.0f, GetTime());
            } else if (LastCarstate == 0) {
                TransmitResetLaneSteering(0.0f, GetTime());
            } else if (LastCarstate == 3) {
                TransmitResetLaneSteering(m_filterProperties.turnRightROISwitch, GetTime());
            } else if (LastCarstate == 4) {
                TransmitResetLaneSteering(-50.0f, GetTime());
            }
            ResetLights();

            //cout << "Carstate 1" << endl;
            //SpeedValue = (tFloat32) 10.0f;
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(m_filterProperties.defaultSpeed, GetTime());
            TransmitDistance(1000.0f, GetTime());

            LastCarstate = 1;
            break;
            //  CarState 2: Go straight
        case 2:
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSteering(0.0f, GetTime());
            TransmitDistance(1.0f, tFalse);
            LastCarstate = 2;
            break;
            //  CarState 3: Rechts abbiegen
        case 3:
            if (m_filterProperties.debugOutput) {
                LOG_INFO(cString("Carstate 3 reached"));
            }
            LastCarstate = 3;
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(m_filterProperties.turnRightSpeed, GetTime());
            TransmitSteering(m_filterProperties.turnRightSteering, GetTime());
            TransmitDistance(m_filterProperties.turnRightDist, tFalse);

            //TransmitDistance(100.0f,GetTime());
            break;
            // CarState 4: Links abbiegen
        case 4:
            LastCarstate = 4;
            ManeuverPhase = 1;
            TransmitSpeed(m_filterProperties.turnLeftSpeed, GetTime());
            TransmitSteering(m_filterProperties.turnLeftSteeringOne, GetTime());
            TransmitDistance(m_filterProperties.turnLeftDistOne, tFalse);
            //TransmitDistance(100.0f,GetTime());
            break;
            //  CarState 5: Parken
        case 5:
            PRINT1("Start Parking. Current ParkingID: %i", currentParkingID);
            TransmitResetDriveDistance(tTrue, GetTime());
            ManeuverPhase = 1;
            TransmitSpeed(0.9f, GetTime());
            TransmitSteering(m_filterProperties.parkingPhaseOneSteering, GetTime());
            // Ruecklicht anmachen und rückwärts fahren
            TransmitRuecklicht(tTrue);
            TransmitDistance(m_filterProperties.parkingPhaseOneDistance, tFalse);
            PRINT1("Finished Parking. Current ParkingID: %i", currentParkingID);
            break;
        case 6: //Zur Kreuzung fahren nach Erkennung Rechts
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(0.5f, GetTime());
            TransmitDistance(DistanceToLine - m_filterProperties.distanceToStopLineRight,
                             tTrue);
            break;
        case 7: //Zur Kreuzung fahren nach Erkennung Links
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(0.5f, GetTime());
            TransmitDistance(m_filterProperties.distanceToStopLineLeft, tTrue);
            break;
        case 8: //Zur Kreuzung fahren nach Erkennung von Haltelinie
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(0.5f, GetTime());
            TransmitDistance(m_filterProperties.distanceToStopLine, tTrue);
            break;
        case 9: //Turn Out Right
            LastCarstate = 9;
            ManeuverPhase = 1;
            //cout << "Phase 1 reached" << endl;
            TransmitSteering(m_filterProperties.steeringZeroCalibration, GetTime());
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(1.0f, GetTime());
            TransmitDistance(m_filterProperties.turnOutRightPhaseOneDistance,
                             tFalse);
            break;
        case 10: //Turn Out Left
            LastCarstate = 10;
            ManeuverPhase = 1;
            TransmitSteering(m_filterProperties.steeringZeroCalibration, GetTime());
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(1.0f, GetTime());
            TransmitDistance(m_filterProperties.turnOutLeftPhaseOneDistance,
                             tFalse);
            break;
        case 11: //Alert (Child detected, Construction site, terror attack)
            TransmitBrakeLight(tTrue);

            // In Kurven sind wir zu langsam. Daher werden wir hier nicht langsamer, wenn wir Kinder sehen.
            if (LastCarstate != 1) {
                break;
            }
            LastCarstate = 11;

            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(m_filterProperties.defaultSpeed * 0.7, GetTime());
            TransmitDistance(1.5f, tFalse);
            break;
        case 12: //Drive slowly, just as one, but slower
            TransmitBrakeLight(tTrue);
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(0.5f, GetTime());
            TransmitDistance(1000.0f, GetTime());
            if (LastCarstate == 9 || LastCarstate == 3 || LastCarstate == 2 || LastCarstate == 10)
                TransmitResetLaneSteering(60.0f, GetTime());
            else if (LastCarstate == 4)
                TransmitResetLaneSteering(0.0f, GetTime());
            break;

        case 13: //Drive to FirstParkingspace
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(0.8f, GetTime());
            // PRINT2("DistanceToLine %f + distanceToParkingspace %f", DistanceToLine, m_filterProperties.distanceToParkingspace)
            TransmitDistance(DistanceToLine + m_filterProperties.distanceToParkingspace, tTrue);
            break;
        case 14: //Drive to NextParkingspace
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitSpeed(0.8f, GetTime());
            // PRINT1("distanceBetweenParkingspaces: %f", m_filterProperties.distanceBetweenParkingspaces);
            TransmitDistance(m_filterProperties.distanceBetweenParkingspaces, tTrue);
            break;

        case 15:
		
            PRINT("Carstate 15 reached");
            TransmitSpeed(m_filterProperties.defaultSpeed, GetTime());
            TransmitResetDriveDistance(tTrue, GetTime());
            TransmitRuecklicht(tTrue);
            TransmitDistance(m_filterProperties.OvertakePhaseZeroDist, tTrue);
            TransmitSteering(m_filterProperties.steeringZeroCalibration, GetTime());
            break;
            // Unbekannter CarState
        default:
            throw invalid_argument("Error: Unknown CarState received!");
            break;
        }
        RETURN_NOERROR;
    }

tTimeStamp BaseControl::GetTime() {
    return (_clock != NULL) ? _clock->GetTime() : cSystem::GetTime();
}
