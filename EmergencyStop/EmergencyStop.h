#ifndef _EMERGENCY_STOP_
#define _EMERGENCY_STOP_

#define OID_USER_ESTOP "adtf.user.EmergencyStop"



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

#include <cmath>


class EmergencyStop : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_USER_ESTOP, "Emergency Stop", adtf::OBJCAT_DataFilter);

public:

	cCriticalSection CritObstacleOut;
	cCriticalSection CritBrakeLightOut;
	cCriticalSection CritSteeringResetOut;
	cCriticalSection CritSpeedOut;	
	cCriticalSection CritSteeringIn;
	cCriticalSection CritPositionIn;
	cCriticalSection CritStopBoolIn;
    cCriticalSection CritOvertakingIn;
	cCriticalSection CritSpeedControllerIn;
	cCriticalSection CritSpeedController2In;
	cCriticalSection CritObstacleToBrainOut;

	/*! The input pin for template data */
	tBool isStopped;
    tBool overtakingPossible;
	cInputPin m_oInputUsStruct;

	std::vector<tBufferID> m_szIdUsStructValues;
	tInt m_szIdUsMinima[1];
	tInt m_szIdUsCurrentIndex;
			
	tBool 	m_bIdsUsStructSet;
	tBool	m_bIdsSpeedInSet;
	tBool	m_bIdsSpeedOutSet;

	//Media Type Descriptions
	cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStruct;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolean;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionObstacleStruct;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionPosition;

	cInputPin	m_oInputSpeedController; //tSignalValue
	tBufferID	m_szIdSpeedInValue;
	tBufferID	m_szIDInputSpeedControllerTss;
	tBool		m_szIDsInputSpeedSet;

	cInputPin	m_oInputSteeringController; //tSignalValue
	tBufferID	m_szIdSteeringInValue;
	tBufferID	m_szIdSteeringInValueTS;
	tBool		m_szIDsInputSteeringSet;

	cOutputPin  m_oOutputSpeedController;
	tBufferID	m_szIDOutputSpeedControllerValue;
	tBufferID	m_szIDOutputSpeedControllerTimeStamp;


	//Stop Boolean
	cInputPin	m_oInputStopBoolean; //tSignalValue
	tBufferID	m_szIdStopBoolean;
	tBufferID	m_szIDInputStopBooleanTS;
	tBool		m_szIDsInputStopBooleanSet;
    
    // Overtaking possible input bool
    cInputPin	m_oOvertakingPossibleBoolean; //tSignalValue
	tBufferID	m_szIdOvertakingPossibleBoolean;
	tBufferID	m_szIDOvertakingPossibleBooleanTS;
	tBool		m_szIDsOvertakingPossibleBooleanSet;

    
    cOutputPin	m_oOutputBrakeLight; //tSignalValue
	tBufferID	m_szIdBrakeLightOutValue;
	tBufferID	m_szIDBrakeLightOutTs;
	tBool		m_szIDBrakeLightSet;

	cOutputPin	m_oOutputObstacle; //m_Output
	tBufferID	m_szIDOutputObstacleXValue;
	tBufferID	m_szIDOutputObstacleYValue;
	tBool		m_szIDsOutputObstacleSet;

	cOutputPin	m_oOutputObstacleFoundToBrain; //m_Output
	tBufferID	m_szIDOutputObstacleFoundToBrainID;
	tBufferID	m_szIDOutputObstacleFoundToBrainIDTs;
	tBool		m_szIDsOutputObstacleFoundToBrainSet;


	cInputPin	m_oInputPosition; //m_Input
	tBufferID	m_szPositionXValue;
	tBufferID	m_szPositionYValue;
	tBufferID	m_szPositionHValue;
	tBool		m_szIDsInputPositionSet;

	cOutputPin	m_oOutputSteeringReset; //tSignalValue
	tBufferID	m_szIdSteeringResetValue;
	tBool		m_szIDSteeringResetSet;

	cCriticalSection m_critSecMinimumUsValue;
    
	//Speichern vom Minmalwert
	tSignalValue m_oMinimumUsValue;
	tInt32 obstacleCounter;
	tFloat32 frontValue;
	tFloat32 desiredSpeed;
		
	// Integer mappings for the order of ultra sonic sensors
	static const int   FRONT_CENTER      = 0;
	static const int 	FRONT_INNER_RIGHT = 1;
	static const int 	FRONT_OUTER_RIGHT = 2;
	static const int 	SIDE_RIGHT		  = 3;
	static const int 	REAR_OUTER_RIGHT  = 4;
	static const int 	REAR_CENTER		  = 5;
	static const int 	REAR_OUTER_LEFT	  = 6;
	static const int 	SIDE_LEFT		  = 7;
	static const int 	FRONT_OUTER_LEFT  = 8;
	static const int 	FRONT_INNER_LEFT  = 9;
	
		

public:
	/*! default constructor for template class
	  \param __info   [in] This is the name of the filter instance.
	*/
	EmergencyStop(const tChar* __info);

	/*! default destructor */
	virtual ~EmergencyStop();
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
	tResult ActivateUsSensors(tFloat32 speed, tFloat32 steering);
	tInt Sgn(tFloat32 nmbr);
	tFloat32 DotProduct(vector<tFloat32> a, vector<tFloat32> b);
	tResult SteeringIntoVector(tFloat32 angle);
	tResult CalcUsWeights(tFloat32 steering, tFloat32 speed);
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	tResult ProcessMinimumValueUs(IMediaSample* pMediaSample, tFloat32 speed, tFloat32 steering);
	tResult ProcessStopBoolean(IMediaSample* pMediaSample);
    tResult ProcessOvertakingPossibleBoolean(IMediaSample* pMediaSample);
	tResult ProcessSpeedController(IMediaSample* pMediaSample);
	tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
	tResult TransmitSteeringReset(tFloat32 offsetValue);
	tResult TransmitBrakeLight(tBool status, tUInt32 time);
	tFloat32 GetMinimaMean(tInt minimaArray[]);
	tUInt32 GetTime();
	tFloat32 Betrag(tFloat32 input);
	tResult TransmitObstacle(tFloat32 xPos, tFloat32 yPos, tFloat32 hPos);
	tResult ProcessPosition(IMediaSample* pMediaSample);
	tResult ProcessSteering(IMediaSample* pMediaSample);
	tFloat32 current_X;
	tFloat32 current_Y;
	tFloat32 current_H;
	tBool m_ObstacleTransmitted;
	tBool resetSteeringSent;
	tBool m_UsActivated[10];
	vector<vector<tFloat32> > m_UsVectors;
	vector<tFloat32> m_SteeringVector;
	tFloat32 current_steering;
	tFloat32 m_MinUsValue;
	tResult TransmitObstacleToBrain(tBool obstacle);
	std::vector<deque<tFloat32> > usQueues;
	tFloat32 ScaleDistanceAccordingToSpeed(tFloat32 distance, tFloat32 speed);
	tFloat32 NormalizeScalingValue(tFloat32 scalingValue);
	
	
public:
	struct filterProperties
	{
		tFloat32 curveAngle;
	
		tFloat32 OvertakingFrontLeftInnerThreshold;
        tFloat32 OvertakingFrontLeftOuterThreshold;
		tFloat32 stopDist;
	
		tFloat32 brakeDist;
		tFloat32 stopCounter;
	
	
		tFloat32 scaleBaseValue;
		tFloat32 scaleMaxValue;
	
		// Skalierungswerte für die Infrarotstrahlen
		tFloat32 usScalerFrontCenter;
		tFloat32 usScalerFrontInner;
		tFloat32 usScalerFrontOuter;
		tFloat32 usScalerSide;
		tFloat32 usScalerBackOuter;
		tFloat32 usScalerBackCenter;
		tInt32	 CounterCeiling;
		tInt32 QueueSize;
		
		tFloat32 usScaler;
		tFloat32 weightScaler;
		tFloat32 gewicht;
		
		tBool debugOutput;
		tBool normalizeSpeedForDebugging;
	};
	/*! the filter properties of this class */
	filterProperties m_filterProperties;
};

//*************************************************************************************************
#endif // _EMERGENCY_STOP_

	/*!
	 *@}
	 */
