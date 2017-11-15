/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/


#ifndef _STATE_CONTROLLER_HEADER
#define _STATE_CONTROLLER_HEADER

#define __guid "adtf.aadc.stateController"

#include "stdafx.h"

#include "juryEnums.h"

/*! @defgroup StateController State Controller
*  @{
*
* This filter implements a state machine that defines the several possible state of the vehicle and manages the communication with the Jury module. This filter can be used by the teams as reference to build their own individual state machine to communicate and respond correctly with the Jury Module.
* \n This filter implements the following states which are defined in juryEnums.h
* \n enum stateCar{
* \n <span style="padding-left: 2em">stateCar_ERROR=-1,</span>
* \n <span style="padding-left: 2em">stateCar_READY=0,</span>
* \n <span style="padding-left: 2em">stateCar_RUNNING=1,</span>
* \n <span style="padding-left: 2em">stateCar_COMPLETE=2,</span>
* \n <span style="padding-left: 2em">stateCar_STARTUP=-2</span>
* \n };
* \n
* \n The Jury uses the following actions to control the vehicle:
* \n enum juryActions{
* \n <span style="padding-left: 2em">action_STOP=-1,</span>
* \n <span style="padding-left: 2em">action_GETREADY=0,</span>
* \n <span style="padding-left: 2em">action_START=1</span>
* \n };
* \n The vehicle transmits the following states to the jury:
* \n enum stateCar{
* \n <span style="padding-left: 2em">stateCar_ERROR=-1,</span>
* \n <span style="padding-left: 2em">stateCar_READY=0,</span>
* \n <span style="padding-left: 2em">stateCar_RUNNING=1,</span>
* \n <span style="padding-left: 2em">stateCar_COMPLETE=2,</span>
* \n <span style="padding-left: 2em">stateCar_STARTUP=-2</span>
* \n };
* \n
* \n The received actions from the jury or the receiving on a sample on the pins Set_State_xxx forces the state machine to change its state.
* \n
* \n After receiving an action_STOP the vehicle has to change to a state from which it can be activated again. Possible would be change to state_STARTUP
* \n
* \n As already mentioned this filter can be used in two different modes:
* \li	The filter automatically reacts on incoming Jury Structs on the inputpin Jury_Struct
* \li	The filter changes its state depending on inputs on the inputpins Set_State_Ready, Set_State_Running, Set_State_Complete, Set_State_Error
*
* \n Mode 1: Jury Structs:
* \n After starting the vehicle is in state state_STARTUP. In this state no DriverStructs are generated as output and the vehicle is waiting for the action action_GETREADY from the Jury. When such an action_GETREADY is received the state changes to state_READY and starts with cyclic sending of DriverStruct with the state state_READY. In this Media Sample the current ManeuverID is also contained, after first changing to state_READY it starts with ID 0.
* \n When the filter receives now an action_START command the filter changes its state to state_RUNNING and sends this state periodically to the Jury Module. At this state change the filter verifies if the ManeuverID of the state_READY command and the action_START command are the same. If it is unequal a warning is emitted but the state is also changed.
* \n If a sample of type tBoolSignalValue with a TRUE in the bValue element is received on the input pin Increment_Maneuver_ID  while being in state state_RUNNING the current ManeuverID is incremented.
* \n If the filter receives now an action_STOP command it changes its state to state_STARTUP and stops transmitting states to the jury.
*
* \n Mode 2: Using the Set_State_xxx pins
* \n Media Samples of type tBoolSignalValue have to be transmitted to the pins Set_State_Ready, Set_State_Running, Set_State_Complete, Set_State_Error, Increment_Maneuver_ID and Restart_Section.
* \n If the sample contains a TRUE in the bValue element the filter reacts as the following list shows:
* \li	Set_State_Ready: change to state_READY
* \li	Set_State_Running: change to state_RUNNING
* \li	Set_State_Complete: change to  state_COMPLETE
* \li	Set_State_Error: change to state_ERROR
* \li	Increment_Maneuver_ID: increments the counter for the ManeuverID
* \li	Restart_Section: resets the ManeuverID to the first ID in the current section.
*
* \n
* \n Note: To perform the last two actions a ManeuverFile has to be set in the Properties.
* \n The Media Samples with the tBoolSignalValues can be generated with the Filter Bool Value Generator.
*
*  \image html StateController.PNG "Plugin State Controller"
*
* \b Dependencies \n
* This plugin needs the following libraries:
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Debug Output to Console<td><td>False
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>Driver_struct<td><td>tDriverStruct
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>Jury_Struct<td>tJuryStruct
* <tr><td>Set_State_Ready<td>tBoolSignalValue
* <tr><td>Set_State_Running<td>tBoolSignalValue
* <tr><td>Set_State_Complete<td>tBoolSignalValue
* <tr><td>Set_State_Error<td>tBoolSignalValue
* <tr><td>Increment_Maneuver_ID<td>tBoolSignalValue
* <tr><td>Restart_Section<td>tBoolSignalValue
* <tr><td>Maneuver_List<td>tManeuverList

* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_StateController
* <tr><td>Filename<td>aadc_stateController.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/

/*! this is the main class of the plugin State Controller */
class cStateController : public adtf::cFilter
{
    /*! set the filter ID and set the version */
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC State Controller", OBJCAT_Tool, "State Controller", 1, 1, 0, "");


public: // construction

    /*! constructor for  class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cStateController(const tChar* __info);

    /*! default destructor*/
    virtual ~cStateController();

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

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

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
    *   \result Returns a standard result code.
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
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*!
    *  The adtf::IRunnable interface declares an object as a callback-entry point.
    *  Overwrite these function when handling system events, thread calls or other
    *  activation calls. Have a look to adtf::IRunnable and  adtf::IKernel.
    *  \n
    *  \n For receiving System Events use within your overwritten cFilter::Init function:
    *  \n
    *  \code _kernel->SignalRegister(static_cast<IRunnable*>(this)); \endcode
    *  \n Your nActivationCode for the Run call will be IRunnable::RUN_SIGNAL. The
    *     pvUserData will be a pointer to adtf::tEventInfo.
    *  \warning IMPORTANT: When registered for signals, never forget to unregister:
    *  \code _kernel->SignalUnregister(static_cast<IRunnable*>(this)); \endcode
    *
    * \param [in] nActivationCode The activation type for running.
    * \param [in] pvUserData pointer to a activation structure depending on the activation type.
    * \param [in] szUserDataSize Size of the activation structure. (in byte)
    * \param [in,out] __exception_ptr Address of variable that points to an IException interface.
    *                                    If not using the cException smart pointer, the interface has to
    *                                    be released by calling Unref()..
    *
    * \result Returns a standard result code.
    */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

    /*! signal for sending the state
    * \param state state to be sent; -1: error, 0: Ready, 1: Running
    * \param maneuverEntry current entry to be sent
    * \return Standard Result Code.
    */
    tResult SendState(stateCar state, tInt16 maneuverEntry);

    /*! creates the timer for the cyclic transmits
    *   \return Standard Result Code.
    */
    tResult createTimer();

    /*! destroys the timer for the cyclic transmits
     *   \return Standard Result Code.
     */
    tResult destroyTimer(__exception = NULL);

    /*! increments the id of the maneuver id by one and updates the list indexes
    *   \return Standard Result Code.
    */
    tResult incrementManeuverID();

    /*! resets the counters to the start of the current section
    *   \return Standard Result Code.
    */
    tResult resetSection();

    /*! changes the state of the car
     * \param newState the new state of the car
     * \return Standard Result Code.
     */
    tResult changeState(stateCar newState);

    /*! set the maneuver id and find the correct indexes
    * \param maneuverId the id of the maneuver which has to be set
    * \return Standard Result Code.
    */
    tResult setManeuverID(tInt maneuverId);

    /*! this functions loads the maneuver list given in the properties
     * \return Standard Result Code.
     */
    tResult loadManeuverList();

    /*! Coder Descriptor for input jury struct */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionJuryStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDJuryStructI8ActionID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;

    /*! Coder Descriptor for output driver struct*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionDriverStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDDriverStructI8StateID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDDriverStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsDriverStructSet;

    /*! Coder Descriptor for input set state error */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSetStateError;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDSetStateErrorbValue;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDSetStateErrorArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSetStateError;

    /*! Coder Descriptor for input set state running*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSetStateRunning;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDSetStateRunningbValue;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDSetStateRunningArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSetStateRunning;

    /*! Coder Descriptor for input set state stop*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSetStateComplete;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDSetStateCompletebValue;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDSetStateArduinoTimestampComplete;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSetStateComplete;

    /*! Coder Descriptor for input set state ready*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSetStateReady;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDSetStateReadybValue;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDSetStateReadyArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSetStateReady;

    /*! Coder Descriptor for input restart section*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRestartSection;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDRestartSectionbValue;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDRestartSectionArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsRestartSection;

    /*! Coder Descriptor for input restart section*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionIncrementManeuver;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDIncrementManeuverbValue;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDIncrementManeuverArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsIncrementManeuver;

    /*! Coder description for maneuver list */
    cObjectPtr<IMediaTypeDescription> m_pDescManeuverList;


    /*! input pin for the run command */
    cInputPin        m_JuryStructInputPin;

    /*! input pin for the set state error command */
    cInputPin        m_StateErrorInputPin;

    /*! input pin for the set state running command */
    cInputPin        m_StateRunningInputPin;

    /*! input pin for the set state stop command */
    cInputPin        m_StateCompleteInputPin;

    /*! input pin for the set state ready command */
    cInputPin        m_StateReadyInputPin;

    /*! input pin for the set state ready command */
    cInputPin        m_StateRestartSectionInputPin;

    /*! input pin for the set state ready command */
    cInputPin        m_StateIncrementManeuverInputPin;
    /*! input pin for the maneuver list*/
    cInputPin        m_ManeuverListInputPin;

    /*! output pin for state from driver */
    cOutputPin        m_DriverStructOutputPin;

    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;

    /*! this is the filename of the maneuver list*/
    cString         m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

    /*! holds the current state of the car */
    stateCar m_state;

    /*! holds the current maneuver id of the car*/
    tInt16 m_i16CurrentManeuverID;

    /*! holds the current index of the maneuvers in the list in the section */
    tInt16 m_i16ManeuverListIndex;

    /*! holds the current index in the lists of sections */
    tInt16 m_i16SectionListIndex;

    /*! handle for the timer */
    tHandle m_hTimer;

    /*! the critical section of the transmit */
    cCriticalSection m_oCriticalSectionTransmit;

    /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerSetup;

};

#endif /** @} */ // end of group
