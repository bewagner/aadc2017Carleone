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
 * $Author:: spiesra $  $Date:: 2017-05-09 08:51:06#$ $Rev:: 62951   $
 **********************************************************************/
#ifndef _ROADSIGN_REACTION_H_
#define _ROADSIGN_REACTION_H_

#define OID_ADTF_ROADSIGN_REACTION "adtf.example.roadsign_reaction"

class cRoadsignReaction: public adtf::cFilter {
	/*! set the filter ID and the version */
	ADTF_FILTER(OID_ADTF_ROADSIGN_REACTION, "RoadsignReaction", adtf::OBJCAT_DataFilter);

protected:

	struct RoadsignStruct {
		tInt16 RoadsignID;
		tFloat32 RoadsignX;
		tFloat32 RoadsignY;
		tFloat32 RoadsignRadius;
		tFloat32 RoadsignDirection;
		tBool Roadsignseen;
	};

	cCriticalSection CritTrafficSignOut;
	cCriticalSection CritFirstPositionReceivedOut;
	cCriticalSection CritZebraOut;
	cCriticalSection CritParkingOut;
	cCriticalSection CritSlowlyOut;
	cCriticalSection CritCrossingOut;
	cCriticalSection CritRechtsvorLinksOut;
	cCriticalSection CritVorfahrtOut;
	cCriticalSection CritStoppingOut;

	cCriticalSection CritRoadsignBoolIn;
	cCriticalSection CritParkingBoolIn;
	cCriticalSection CritCarPositionIn;
	cCriticalSection CritRoadsignExtIn;

	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalInt;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalBool;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionPos;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionTrafficSign;

	//InputPins
	cInputPin m_RoadsignExtInput;
	tBufferID m_RoadsignExtBufferID;
	tBufferID m_RoadsignExtRvec0BufferID;
	tBufferID m_RoadsignExtRvec1BufferID;
	tBufferID m_RoadsignExtRvec2BufferID;
	tBufferID m_RoadsignExtTvec0BufferID;
	tBufferID m_RoadsignExtTvec1BufferID;
	tBufferID m_RoadsignExtTvec2BufferID;
	tBool m_RoadsignExtBufferSet;

	cInputPin m_CarPostionInput;
	cInputPin m_CarPositionInput;
	tBufferID m_CarPositionXBufferID;
	tBufferID m_CarPositionYBufferID;
	tBufferID m_CarPositionDirectionBufferID;
	tBool m_CarPositionBufferSet;

	cInputPin m_RoadsignBoolInput;
	tBufferID m_RoadsignBoolBufferID;
	tBufferID m_RoadsignBoolBufferIDTs;
	tBool m_RoadsignBoolBufferSet;

	cInputPin m_ParkingBoolInput;
	tBufferID m_ParkingBoolBufferID;
	tBufferID m_ParkingBoolBufferIDTs;
	tBool m_ParkingBoolBufferSet;

	//OutputPins
	cOutputPin m_oOutputStopping;
	tBufferID m_szIdOutputStopping;
	tBufferID m_szIdOutputStoppingTs;
	tBool m_szIdOutputStoppingSet;

	cOutputPin m_oOutputZebraSign;
	tBufferID m_szIdOutputZebraSign;
	tBufferID m_szIdOutputZebraSignTs;
	tBool m_szIdOutputZebraSignSet;

	cOutputPin m_oOutputVorfahrt;
	tBufferID m_szIdOutputVorfahrt;
	tBufferID m_szIdOutputVorfahrtTs;
	tBool m_szIdOutputVorfahrtSet;

	cOutputPin m_oOutputRechtsVorLinks;
	tBufferID m_szIdOutputRechtsVorLinks;
	tBufferID m_szIdOutputRechtsVorLinksTs;
	tBool m_szIdOutputRechtsVorLinksSet;

	cOutputPin m_oOutputCrossing;
	tBufferID m_szIdOutputCrossing;
	tBufferID m_szIdOutputCrossingTs;
	tBool m_szIdOutputCrossingSet;

	cOutputPin m_oOutputSlowly;
	tBufferID m_szIdOutputSlowly;
	tBufferID m_szIdOutputSlowlyTs;
	tBool m_szIdOutputSlowlySet;

	cOutputPin m_oOutputParking;
	tBufferID m_szIdOutputParking;
	tBufferID m_szIdOutputParkingTs;
	tBool m_szIdOutputParkingSet;

	cOutputPin m_oOutputFirstPositionReceived;
	tBufferID m_szIdFirstPositionReceived;
	tBufferID m_szIdFirstPositionReceivedTs;
	tBool m_szIDFirstPositionReceivedSet;

	cOutputPin m_oRoadsignOutput;
	tBufferID m_oRoadsignIDOutputBufferID;
	tBufferID m_oRoadsignXOutputBufferID;
	tBufferID m_oRoadsignYOutputBufferID;
	tBufferID m_oRoadsignDirectionOutputBufferID;
	tBool m_oRoadsignOutputBufferSet;

	std::vector<RoadsignStruct> seenRoadsigns;

	tFloat32 currentCarPositionX;
	tFloat32 currentCarPositionY;
	tFloat32 currentCarDirection;

	tFloat32 currentRvec[3];
	tFloat32 currentTvec[3];

	RoadsignStruct currentRoadsign;

	tBool FirstPositionReceived;
	tBool RoadsignBool;
	tBool ParkingBool;
	tBool configLoaded;

	tFloat32 DEG2RAD;

	/*! storage for the roadsign data */
	vector<RoadsignStruct> m_roadSigns;

public:
	/*! default constructor for template class
	 \param __info   [in] This is the name of the filter instance.
	 */
	cRoadsignReaction(const tChar* __info);

	/*! default destructor */
	virtual ~cRoadsignReaction();

	struct filterProperties {
		tBool debugOutput;
		tFloat32 RoadsignToRoadsignDist;
		tFloat32 neighborhoodDist;
		tFloat32 TransmitCrossingDistance;
		tFloat32 TransmitSlowDistance;
	};

	filterProperties m_filterProperties;

protected:
	/*! Implements the default cFilter state machine call. It will be
	 called automatically by changing the filters state and needs
	 to be overwritten by the special filter.
	 Please see page_filter_life_cycle for further information on when the state of a filter changes.
	 \param [in,out] __exception   An Exception pointer where exceptions will be put when failed.
	 If not using the cException smart pointer, the interface has to
	 be released by calling Unref().
	 \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
	 \return Standard Result Code.
	 */
	tResult Init(tInitStage eStage, __exception);

	/*! Implements the default cFilter state machine call. It will be
	 called automatically by changing the filters state and needs
	 to be overwritten by the special filter.
	 Please see  page_filter_life_cycle for further information on when the state of a filter changes.
	 \param  eStage [in]    The Init function will be called when the filter state changes as follows:\n
	 \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
	 If not using the cException smart pointer, the interface has to
	 be released by calling Unref().
	 \return Standard Result Code.
	 */
	tResult Shutdown(tInitStage eStage, __exception);

	/*! This Function will be called by all pins the filter is registered to.
	 \param [in] pSource Pointer to the sending pin's IPin interface.
	 \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
	 \param [in] nParam1 Optional integer parameter.
	 \param [in] nParam2 Optional integer parameter.
	 \param [in] pMediaSample Address of an IMediaSample interface pointers.
	 \return   Returns a standard result code.
	 \warning This function will not implement a thread-safe synchronization between the calls from different sources.
	 You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
	 */
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

	//tResult ProcessRoadsign(IMediaSample* pMediaSample);

	tResult ProcessRoadsignExt(IMediaSample* pMediaSample);

	tResult ProcessCarPosition(IMediaSample* pMediaSample);

	tResult ProcessRoadsignBool(IMediaSample* pMediaSample);

	tResult ProcessParkingBool(IMediaSample* pMediaSample);

	tResult TransmitStopping(tBool stopBool);

	tResult TransmitVorfahrt(tBool bValue);

	tResult TransmitRechtsVorLinks(tBool bValue);

	tResult TransmitCrossing(tBool bValue);

	tResult TransmitSlowly(tBool value);

	tResult TransmitParking(tBool parkingBool);

	tResult TransmitFirstPositionReceived(tBool brakeLight);

	tResult TransmitTrafficSign(RoadsignStruct currentRoadsign);

	tResult CheckNeighborhood();

	tResult ResetRoadsignseenBool();

	tResult LoadConfiguration();

	tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);

	tFloat32 mod(tFloat32 x, tFloat32 y);

	tResult TransmitZebraSign(tBool ZebraSignBool);

	tTimeStamp GetTime();

	tFloat32 Betrag(tFloat32 input);

	tResult PropertyChanged(const tChar* strName);

};

//*************************************************************************************************
#endif // _TEMPLATE_FILTER_H_

/*!
 *@}
 */
