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

#ifndef CCARCONTROLLER_H
#define CCARCONTROLLER_H

class Widget;

#define OID_ADTF_FILTER_DEF                "adtf.aadc.CarController"
#define ADTF_FILTER_DESC                   "AADC Car Controller"
#define ADTF_FILTER_VERSION_SUB_NAME       "Car Controller"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL   "AADC CarController"
#define ADTF_FILTER_VERSION_STRING         "1.0.0"
#define ADTF_FILTER_VERSION_Major          1
#define ADTF_FILTER_VERSION_Minor          0
#define ADTF_FILTER_VERSION_Build          0
#define ADTF_FILTER_VERSION_LABEL          "Car Controller For AADC."
#define ADTF_CATEGORY OBJCAT_Application

/*! @defgroup CarController Car Controller
*  @{
*
* With this filter the basic functions of the car can be controlled. You can set the steering and the
* speed controller with the two bars or use the keys described in the window. To use the keys the
* focus has to be set on this window by clicking in the window or the title bar. With the buttons at
* the bottom the lights of the car can be switched on or off.
*
*  \image html CarController.PNG "Plugin Car Controller"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li QT  v.4.7.1
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>SpeedController<td><td>tSignalValue
* <tr><td>SteeringController<td><td>tSignalValue
* <tr><td>headLightEnabled<td><td>tBoolSignalValue
* <tr><td>breakLightEnabled<td><td>tBoolSignalValue
* <tr><td>reverseLightEnabled<td><td>tBoolSignalValue
* <tr><td>turnSignalLeftEnabled<td><td>tBoolSignalValue
* <tr><td>turnSignalRightEnabled<td><td>tBoolSignalValue
* <tr><td>USSFrontEnabled<td><td>tBoolSignalValue
* <tr><td>USSRearEnabled<td><td>tBoolSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_CarController
* <tr><td>Filename<td>aadc_CarController.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! this is the main class of the car controller filter */
class cCarController : public QObject, public cBaseQtFilter
{
    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
    ADTF_FILTER_VERSION(
        OID_ADTF_FILTER_DEF,
        ADTF_FILTER_DESC,
        ADTF_CATEGORY,
        ADTF_FILTER_VERSION_SUB_NAME,
        ADTF_FILTER_VERSION_Major,
        ADTF_FILTER_VERSION_Minor,
        ADTF_FILTER_VERSION_Build,
        ADTF_FILTER_VERSION_LABEL);

    Q_OBJECT

public:
    /*! constructor for template class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cCarController(const tChar* __info);

    /*! Destructor */
    virtual ~cCarController();


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

private:
    /*! creates all the output Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*!
     * Transmits bool value.
     *
     * \param [in,out]  pin         outputpin to use
     * \param           value       value to transmit
     * \param           timestamp   arduino timestamp to use
     *
     * \return standard adtf error code
     */
    tResult TransmitBoolValue(cOutputPin* pin, tBool value, tUInt32 timestamp);

    /*!
     * Transmits float value.
     *
     * \param [in,out]  pin         outputpin to use
     * \param           value       The value to send
     * \param           timestamp   arduino timestamp to use
     *
     * \return  standard adtf error code
     */
    tResult TransmitFloatValue(cOutputPin* pin, tFloat32 value, tUInt32 timestamp);


    /*! the media description for bool values */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;

    /*! the media description for float values */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionFloat;


    /*! The output pin for speed controller */
    cOutputPin     m_oOutputSpeedController;
    /*! The output pin for steering controller */
    cOutputPin     m_oOutputSteeringController;
    /*! The output pin for head light  */
    cOutputPin     m_oOutputHeadLight;
    /*! The output pin for reverse light */
    cOutputPin     m_oOutputReverseLight;
    /*! The output pin for brake light */
    cOutputPin     m_oOutputBrakeLight;
    /*! The output pin for turn right controller */
    cOutputPin     m_oOutputTurnRight;
    /*! The output pin for turn left controller */
    cOutputPin     m_oOutputTurnLeft;
    /*! The output pin for turn left controller */
    cOutputPin     m_oOutputHazzard;
    /*! The output pin for uss enable front controller */
    cOutputPin     m_oOutputUssEnableFront;
    /*! The output pin for uss enable read controller */
    cOutputPin     m_oOutputUssEnableRear;

private:
    /*! Creates the widget instance
    *   \return handle to view
    */
    tHandle CreateView();
    /*! Destroys the widget instance
    *   \result Returns a standard result code.
    */
    tResult ReleaseView();

    /*! The widget of qt window*/
    Widget* m_pWidget;


    /*! The critical section transmit control */
    cCriticalSection m_critSecTransmitControl;

    /*! The critical section transmit bool */
    cCriticalSection m_critSecTransmitBool;

public slots:

    /*!
     * Sends a steering to output pin
     *
     * \param   value   The value.
     */
    void SendSteering(float value);

    /*!
     * Sends a throttle to output pin
     *
     * \param   value   The value.
     */
    void SendThrottle(float value);

    /*!
     * Toggle lights to output pin
     *
     * \param   buttonId    Identifier for the button.
     */
    void ToggleLights(int buttonId);

    /*!
     * Toggle ultrasonic to  output pin
     *
     * \param   buttonId    Identifier for the button.
     */
    void ToggleUs(int buttonId);

};
#endif /** @} */ // end of group
