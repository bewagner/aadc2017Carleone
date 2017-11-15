/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef _CONTROLLER_PID_H_
#define _CONTROLLER_PID_H_

#define OID_ADTF_PIDCONTROLLER "adtf.aadc.controller"

/*! @defgroup ControllerPID Controller PID
*  @{
*
* This is filter is an prototyp for an PID Controller
*
*  \image html ControllerPID.PNG "Plugin Controller PID"
*
* \b Dependencies \n
* This plugin needs the following libraries:
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Controller Kd value<td><td>1
* <tr><td>Controller Ki value<td><td>1
* <tr><td>Controller Kp value<td><td>1
* <tr><td>Controller Typ<td><td>p
* <tr><td>Sample Interval [msec]<td><td>1
* <tr><td>use automatically calculated sample interval<td><td>True
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>manipulated variable<td>the output pin for the manipulated value<td>tSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>measured variable<td>the input pin for the measured value<td>tSignalValue
* <tr><td>set point<td>the input pin for the set point value<td>tSignalValue
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/algorithms/AADC_ControllerPID
* <tr><td>Filename<td>aadc_controllerPID.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! this is the main class of the controller pid plugin */
class cControllerPID : public adtf::cFilter
{
    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_PIDCONTROLLER, "AADC PID Controller", OBJCAT_DataFilter, "Controller Filter", 1, 0,0, "Beta Version");

    /*! the input pin for the measured value */
    cInputPin m_oInputMeasured;
    /*! the input pin for the set point value */
    cInputPin m_oInputSetPoint;
    /*! the output pin for the manipulated value */
    cOutputPin m_oOutputManipulated;


    cCriticalSection CritMeasuredIn;
    cCriticalSection CritSetIn;
    cCriticalSection CritAdjustedOut;
public:
    /*! constructor for class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cControllerPID(const tChar* __info);

    /*! Destructor. */
    virtual ~cControllerPID();
    tResult PropertyChanged(const tChar* strName);
protected: // overwrites cFilter


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
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:
    /*! creates all the output Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! creates all the input Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! calculates the manipulated value for the given values, it uses the setpoint in m_f32SetPoint
    * \param f32MeasuredValue    the measuredValue
    * \return the controller value
    */
    tFloat32 getControllerValue(tFloat32 f32MeasuredValue);

    /*!
    * Gets the time.
    *
    * \return  The streamtime in milliseconds
    */
    tTimeStamp GetTime();

    /*! holds the last measuredValue */
    tFloat32 m_f32MeasuredVariable;
    /*! holds the last measured error */
    tFloat32 m_f32LastMeasuredError;
    /*! holds the last setpoint */
    tFloat32 m_f32SetPoint ;
    /*! holds the last sample time */
    tTimeStamp m_lastSampleTime;
    /*! holds the accumulatedVariable for the controller*/
    tFloat32 m_f32AccumulatedVariable;

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignal;
    /*! the id for the f32value of the media description for the signal value input pins */
    tBufferID m_szIDSignalF32Value;
    /*! the id for the arduino time stamp of the media description for the signal value input pins */
    tBufferID m_szIDSignalArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSignalSet;

    struct filterProperties {
    	tFloat32 pValue;
    	tFloat32 iValue;
    	tFloat32 dValue;
    } m_filterProperties;

};
/*! @} */ // end of group
#endif // _CONTROLLER_PID_H_

