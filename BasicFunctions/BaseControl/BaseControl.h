#ifndef _BASE_CONTROL_
#define _BASE_CONTROL_

#define OID_USER_BASECONTROL "adtf.user.BaseControl"
#include <unistd.h>



/*! @defgroup TemplateFilter
 *  @{
 *
 *  \image html User_Template.PNG "Plugin Template Filter"
 *
 * This is a small template which can be used by the AADC teams for their own filter implementations.
 * \b Dependencies \n
 * This plugin needs the following libraries:
 *
 *
 * <b> Filter Properties</b>
 * <table>
 * <tr><th>Property<th>Description<th>Default
 * </table>
 *
 * <b> Output Pins</b>
 * <table>
 * <tr><th>Pin<th>Description<th>MajorType<th>SubType
 * <tr><td>output_template<td>An example output pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
 *</table>
 *
 * <b> Input Pins</b>
 * <table>
 * <tr><th>Pin<th>Description<th>MajorType<th>SubType
 * <tr><td>input_template<td>An example input pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
 * </table>
 *
 * <b>Plugin Details</b>
 * <table>
 * <tr><td>Path<td>src/aadcUser/AADC_TemplateFilter
 * <tr><td>Filename<td>user_templateFilter.plb
 * <tr><td>Version<td>1.0.0
 * </table>
 *
 *
 */

//!  Template filter for AADC Teams
/*!
 * This is a example filter for the AADC
 */
class BaseControl : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_USER_BASECONTROL, "Base Control", adtf::OBJCAT_DataFilter);
	


public:
	
	// Struct für Parkplätze
	struct ParkingspaceStruct 
	{
		tInt16		ParkingspaceID;
		tFloat32	ParkingspaceX;
		tFloat32	ParkingspaceY;
		tBool       ParkingspaceStatus;
		tFloat32	ParkingspaceDirection;
		tBool Valid;

		bool operator<(const ParkingspaceStruct& input) const {
			return input.ParkingspaceID < this->ParkingspaceID;
		}
	};

    /*! the input pin for template data */
	tBool isStopped;
	tBool	m_bIdsSpeedInSet;
	tBool	m_bIdsSpeedOutSet;
	tBool	TurnInProcess;
	tBool 	OverallDistanceSet;
	tFloat32 DistanceAtStartOfTurn;
	tInt ManeuverPhase;
	tFloat32 StopDistance;
	tBool LaneKeepingActive;
	ParkingspaceStruct lastParkingspace;
	
	cCriticalSection m_critSecSteeringInput;
	cCriticalSection m_critSecPositionInput;
	cCriticalSection m_critSecUltrasonicInput;
	cCriticalSection m_critSecParkingIDInput;
	cCriticalSection m_critSecDistanceToLineInput;
	cCriticalSection m_critSecFinishedInput;
	cCriticalSection m_critSecCarStateInput;
	cCriticalSection m_critSecStopBool;
	cCriticalSection m_critSecOverallDistanceInput;

	cCriticalSection CritParkingOut;
	cCriticalSection CritDriveDIstanceOut;
	cCriticalSection CritSpeedOut;
	cCriticalSection CritSteeringOut;
	cCriticalSection CritResetDriveDistance;
	cCriticalSection CritManeuverFinishedOut;
    cCriticalSection CritBlinkerLinksOut;
    cCriticalSection CritBlinkerRechtsOut;
    cCriticalSection CritRuecklichtOut;
	cCriticalSection CritBrakeLightOut;
	cCriticalSection CritResetLaneSteering;






	//Media Type Descriptions
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue2;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue3;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue4;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue5;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue6;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue7;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue8;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolean;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolean2;
	cObjectPtr<IMediaTypeDescription> m_pDescDriverStruct;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStruct;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionPosition;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionParking;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValueInt;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionDriveDistance;
	
	tFloat32 DEG2RAD;

	cInputPin 	m_InputUsStruct;
	tBufferID	m_ParkinSpaceDepthValueID;
	tBool		m_InputUsSet;
	tFloat32 	m_ParkinSpaceDepthValue;

	cInputPin	m_oInputPosition; //m_Input
	tBufferID	m_szPositionXValue;
	tBufferID	m_szPositionYValue;
	tBufferID	m_szPositionHValue;
	tBool		m_szIDsInputPositionSet;
	
	cInputPin	m_oInputParkingID; //m_Input;
	tBufferID	m_szParkingID;
	tBufferID	m_szParkingIDTs;
	tBool		m_szIDsInputParkingIDSet;

	cInputPin	m_DistanceToLine; //tSignalValue
	tBufferID	m_DIstanceToLineBuffer;
	tBufferID	m_DIstanceToLineBufferTs;
	tBool		m_DistanceToLineSet;

	cOutputPin	m_oOutputDriveDistance; //m_Output
	tBufferID	m_szDriveDistanceOutValue;
	tBufferID	m_szIDOutputDriveDistanceTss;
	tBool		m_szIDsOutputDriveDistanceSet;

	cOutputPin	m_oOutputParkingspace; //m_Output
	tBufferID	m_szIDOutputParkingXValue;
	tBufferID	m_szIDOutputParkingYValue;
	tBufferID	m_szIDOutputParkingIDValue;
	tBufferID	m_szIDOutputParkingStatusValue;
	tBool		m_szIDsOutputParkingspaceSet;


	cInputPin	m_oInputDistanceSinceLastSample; //tSignalValue
	tBufferID	m_szIdDistanceInValue;
	tBufferID	m_szIDDistanceTs;
	tBool		m_szIDsInputDistanceSet;

	cOutputPin	m_oOutputDebug; //tSignalValue
	tBufferID	m_szIdDebugValue;
	tBufferID	m_szIDDebugTs;
	tBool		m_szIDsDebugSet;

	cInputPin	m_oInputLaneSteering; //tSignalValue
	tBufferID	m_szIdSteeringInValue;
	tBufferID	m_szIDInputSteeringControllerTs;
	tBool		m_szIDsInputSteeringSet;

	cInputPin	m_oInputCarState; //tSignalValue
	tBufferID	m_szIdCarStateInValue;
	tBufferID	m_szIDCarStateInTs;
	tBool		m_szIDsCarStateInSet;

	//Driver STruct Input
	cInputPin    m_DriverStructInputPin;
	tBufferID	m_szIdDriverStructInValue;
	tBufferID	m_szIDDriverStructInID;
	tBool		m_szIDsDriverStructSet;

	//Stop Boolean
	cInputPin	m_oInputStopBoolean; //tSignalValue
	tBufferID	m_szIdStopBoolean;
	tBufferID	m_szIDInputStopBooleanTS;
	tBool		m_szIDsInputStopBooleanSet;

	cOutputPin	m_oOutputResetDriveDistance; //tSignalValue
	tBufferID	m_szIdResetDriveDistance;
	tBufferID	m_szIdResetDriveDistanceTs;
	tBool		m_szIDResetDriveDistanceSet;

	cOutputPin	m_oOutputResetLaneSteering; //tSignalValue
	tBufferID	m_szIdResetLaneSteering;
	tBufferID	m_szIdResetLaneSteeringTs;
	tBool		m_szIDResetLaneSteeringSet;


	//Finished Boolean
	cInputPin	m_oInputFinished; //tSignalValue
	tBufferID	m_szIdFinished;
	tBufferID	m_szIDFinishedTS;
	tBool		m_szIDsFinishedSet;

	/*! the output pin for template data */
	cOutputPin  m_oOutputSpeedController; // tSignalValue
	tBufferID	m_szIDOutputSpeedControllerValue;
	tBufferID	m_szIDOutputSpeedControllerTimeStamp;
	tBool		m_szIDsOutputSpeedSet;

	cOutputPin  m_oOutputSteering; // tSignalValue
	tBufferID	m_szIDOutputSteeringValue;
	tBufferID	m_szIDOutputSteeringTimeStamp;
	tBool		m_szIDsOutputSteeringSet;

	cOutputPin  m_oOutputFinished; // tSignalValue
	tBufferID	m_szIDOutputFinishedValue;
	tBufferID	m_szIDOutputFinishedTimeStamp;
	tBool		m_szIDsOutputFinishedSet;

    // Blinker links
    cOutputPin  m_oOutputBlinkerLinks; // tSignalValue
	tBufferID	m_szIDOutputBlinkerLinksValue;
	tBufferID	m_szIDOutputBlinkerLinksTimeStamp;
	tBool		m_szIDsOutputBlinkerLinksSet;
    
    // Blinker rechts
    cOutputPin  m_oOutputBlinkerRechts; // tSignalValue
	tBufferID	m_szIDOutputBlinkerRechtsValue;
	tBufferID	m_szIDOutputBlinkerRechtsTimeStamp;
	tBool		m_szIDsOutputBlinkerRechtsSet;
    
    // Rücklicht
    cOutputPin  m_oOutputRuecklicht; // tSignalValue
	tBufferID	m_szIDOutputRuecklichtValue;
	tBufferID	m_szIDOutputRuecklichtTimeStamp;
	tBool		m_szIDsOutputRuecklichtSet;

    cOutputPin  m_oOutputBrakeLight; // tSignalValue
	tBufferID	m_szIDOutputBrakeLightValue;
	tBufferID	m_szIDOutputBrakeLightTimeStamp;
	tBool		m_szIDsOutputBrakeLightSet;

	cOutputPin	m_SwitchLaneOut;
	tBufferID	m_SwitchLaneBuffer;
	tBufferID	m_SwitchLaneBufferTs;
	tBool		m_SwitchLaneBufferSet;
	cCriticalSection CritSwitchLaneOut;

	tFloat32 SteeringValue;
	tTimeStamp SteeringTimestamp;
	tTimeStamp SpeedTimestamp;
	tFloat32 SpeedValue;

	tInt calibCounter;
	tInt parkingCounter;

	tBool parkingSpacesXMLLoaded;
	tBool parkingSpacesXMLInputValid;

	cCriticalSection m_critSecMinimumUsValue;
	//Speichern vom Minmalwert
	tSignalValue m_oMinimumUsValue;

	tUInt32 CurrentCarstate; //0 = waiting, 1 = normal, 2 = go straight, 3 = turn right, 4 = turn left, 5 = parking
	tUInt32 LastCarstate;
	tInt16 ManeuverID;
	tFloat32 DistanceTravelled;
	tFloat32 DistanceToLine;
	tFloat32 current_X;
	tFloat32 current_Y;
	tFloat32 current_H;
	tInt16 parking_ID;
	tUInt16 parking_Status;
	tInt16 currentParkingID;
	tInt16 currentParkingspaceIndex;
	
	   

public:
	/*! default constructor for template class
	  \param __info   [in] This is the name of the filter instance.
	*/
	BaseControl(const tChar* __info);

	/*! default destructor */
	virtual ~BaseControl();
	tResult PropertyChanged(const tChar* strName);
protected:
	/*! Implements the default cFilter state machine call. It will be
	 *	    called automatically by changing the filters state and needs
	 *	    to be overwritten by the special filter.
	 *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
	 *
	 *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
	 *        If not using the cException smart pointer, the interface has to
	 *        be released by calling Unref().
	 *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
	 *    \return Standard Result Code.
	 */
	tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
	
	tResult ProcessParkingID(IMediaSample* pMediaSample);
	
	// Einlesen der .xml für die Parkplätze
	tResult LoadConfiguration();

	// Suchen der ID des nächsten (kleinste Distanz) Parkplatzes
	ParkingspaceStruct	GetClosestParkingspace(std::vector<set<ParkingspaceStruct> > );
	// Berechnet die Distanz zwischen zwei 2D-Punkten.
	tFloat32 Distance(tFloat32 x1, tFloat32 y1, tFloat32 x2, tFloat32 y2);
	tFloat32 Distance(ParkingspaceStruct a,  ParkingspaceStruct b);
	
	/*!
	 *   Implements the default cFilter state machine call. It will be
	 *   called automatically by changing the filters state and needs
	 *   to be overwritten by the special filter.
	 *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
	 *
	 *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
	 *                                   If not using the cException smart pointer, the interface has to
	 *                                   be released by calling Unref().
	 *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
	 *   \return Returns a standard result code.
	 *
	 */
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

	/*! This Function will be called by all pins the filter is registered to.
	 *   \param [in] pSource Pointer to the sending pin's IPin interface.
	 *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
	 *   \param [in] nParam1 Optional integer parameter.
	 *   \param [in] nParam2 Optional integer parameter.
	 *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
	 *   \return   Returns a standard result code.
	 *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
	 *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
	 */
	ParkingspaceStruct GetClosestParkingspaceByID(int currentParkingspaceID);
	
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	tResult ResetLights();
	tResult ProcessStopBoolean(IMediaSample* pMediaSample);
	tResult ProcessDistanceToLine(IMediaSample* pMediaSample);
	tResult ProcessDistanceSinceLastSample(IMediaSample* pMediaSample);
	tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
	tResult ProcessSteering(IMediaSample* pMediaSample);
	tResult ProcessUS(IMediaSample* pMediaSample);
	tResult ProcessFinished(IMediaSample* pMediaSample);
	tResult TransmitFinished(tBool value);
    tResult TransmitBlinkerLinks(tBool value);
    tResult TransmitBlinkerRechts(tBool value);
    tResult TransmitRuecklicht(tBool value);
	tResult TransmitSteering(tFloat32 speed, tUInt32 timestamp);
	tResult ProcessDistance(IMediaSample* pMediaSample);
	tResult TransmitDebug(tFloat32 speed, tUInt32 timestamp);
	tResult TransmitResetDriveDistance(tBool boolean, tUInt32 timestamp);
	tResult TransmitResetLaneSteering(tFloat32 dynWidth, tUInt timestamp);
	tResult ProcessCarState(IMediaSample* pMediaSample);
	tResult TransmitDistance(tFloat32 distance, tBool brake);
	tResult ActOnCarState(tUInt32 carstate);
	tResult TransmitBrakeLight(tBool boolVal);
	tResult ProcessPosition(IMediaSample* pMediaSample);
	tResult TransmitParking(tInt16 Identifier,tFloat32 xPos, tFloat32 yPos, tUInt16 Status, tUInt32 CurrentCarstate);
	tResult TransmitSwitchLane(tBool value);
    tInt32 GetParkingspaceSetIndex(tInt32 parkingspaceIndex);
    ParkingspaceStruct SetAt(set<ParkingspaceStruct> s, int index);
    tTimeStamp GetTime();
public:
	


    std::vector<std::set<ParkingspaceStruct> > parkingSpaceList;
	

    struct filterProperties
    {
        tBool debugOutput;
        tFloat32 mindist;

        tFloat32 turnLeftDistOne;
        tFloat32 turnLeftDistTwo;
        tFloat32 turnLeftSteeringOne;
        tFloat32 turnLeftSteeringTwo;

        tFloat32 turnRightSteering;
        tFloat32 turnRightDist;
        tFloat32 turnRightDistTwo;
        tFloat32 turnRightSteeringTwo;

        tFloat32 defaultSpeed;
        tFloat32 turnRightSpeed;
        tFloat32 turnLeftSpeed;
        tFloat32 distanceToStopLineRight;
        tFloat32 distanceToStopLineLeft;
        tFloat32 distanceToStopLine;
        tFloat32 steeringZeroCalibration;

        tFloat32 turnOutRightPhaseOneDistance;
        tFloat32 turnOutRightPhaseTwoDistance;
        tFloat32 turnOutRightPhaseThreeDistance;
        tFloat32 turnOutRightPhaseTwoSteering;
        tFloat32 turnOutRightPhaseThreeSteering;
        tFloat32 turnOutRightPhaseFourDistance;

        tFloat32 turnOutLeftPhaseOneDistance;
        tFloat32 turnOutLeftPhaseTwoDistance;
        tFloat32 turnOutLeftPhaseThreeDistance;
        tFloat32 turnOutLeftPhaseTwoSteering;
        tFloat32 turnOutLeftPhaseThreeSteering;

        tFloat32 parkingPhaseOneDistance;
        tFloat32 parkingPhaseTwoDistance;
        tFloat32 parkingPhaseThreeDistance;
        tFloat32 parkingPhaseFourDistance;
        tFloat32 parkingPhaseOneSteering;
        tFloat32 parkingPhaseTwoSteering;
        tFloat32 parkingPhaseThreeSteering;
        tFloat32 parkingPhaseFourSteering;

        tFloat32 distanceToParkingspace;
        tFloat32 distanceBetweenParkingspaces;

        tFloat32 OvertakePhaseZeroDist;
        tFloat32 OvertakePhaseOneDist;
        tFloat32 OvertakePhaseOneSteering;
        tFloat32 OvertakePhaseTwoDist;
        tFloat32 OvertakePhaseTwoSteering;
        tFloat32 OvertakePhaseThreeDist;
        tFloat32 OvertakePhaseThreeSteering;
        tFloat32 OvertakePhaseFourDist;
        tFloat32 OvertakePhaseFourSteering;
        tFloat32 OvertakeDist;


        tFloat32 turnRightROISwitch;

    };
    /*! the filter properties of this class */
    filterProperties m_filterProperties;
};

//*************************************************************************************************
#endif // _BASE_CONTROL_

/*!
 *@}
 */
