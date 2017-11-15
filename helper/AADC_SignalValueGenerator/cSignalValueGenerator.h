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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef _SIGNAL_VALUE_HEADER
#define _SIGNAL_VALUE_HEADER

/*! value struct which is read from the file*/
struct tValueStruct
{
    /*! the timestamp in milliseconds */
    tFloat32 timeStamp;
    /*! value first order */
    tFloat32 f32Value1;
    /*! value second order */
    tFloat32 f32Value2;
};

#define __guid "adtf.aadc.signalValueGenerator"

#include "stdafx.h"
#include "displaywidget.h"

/*! @defgroup SignalValueGenerator Signal Value Generator
*  @{
*
* This filter can be used to generate two waveforms in Media Samples of the type tSignalValue.
* \n The waveforms can be specified in table in the GUI of the filter by specify a timestamp and corresponding two values which are transmitted on the output pins. The filter does a linear interpolation between the given points and the sample rate of the output samples must be set in the property Actuator Update Rate [Hz] .
* \n The filter can also work with a default file with predefined values. The file can be selected in the properties and the values can be loaded with the button Load Defaults. After editing values the button Save can be used to save the values to a new file or also back to the file with the default values used in the properties.
* \n The values in the properties Default Value 1 and Default Value 2 are transmitted always except when the defined waveform is transmitted, i.e. before the waveform starts and after the waveform ends.
* \n The file must be a simple file containing in one line first the timestamp, followed by the value 1 and value 2. A file looks like:
* \n <span style="padding-left: 2em">1000</span> <span style="padding-left: 2em"> 0</span> <span style="padding-left: 2em">80</span>
* \n <span style="padding-left: 2em">8000</span> <span style="padding-left: 2em"> 3</span> <span style="padding-left: 2em">110</span>
* \n <span style="padding-left: 2em">15000</span><span style="padding-left: 2em">3</span> <span style="padding-left: 2em">90</span>
*
*  \image html SignalValueGenerator.PNG "Plugin Signal Value Generator"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li QT  v.4.7.1
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Actuator Update Rate [Hz]<td>Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)<td>30
* <tr><td>Default Value 1<td>Defines the default value which is transmitted when nothing is read from the list<td>0
* <tr><td>Default Value 2<td>Defines the default value which is transmitted when nothing is read from the list<td>0
* <tr><td>File for Default Values<td>Set the file with the default files here<td>
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>Value_1<td><td>tSignalValue
* <tr><td>Value_2<td><td>tSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_SignalValueGenerator
* <tr><td>Filename<td>aadc_signalValueGenerator.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/

/*! this is the main class of the signal value generator plugin
*/
class cSignalValueGenerator : public QObject, public cBaseQtFilter
{
    /*! declare the filter version */
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Signal Value Generator", OBJCAT_Auxiliary, "Signal value Generator filter", 1, 0, 1, "");

    Q_OBJECT


public:

    /*! default constructor
    * \param __info info argument for filter
    */
    cSignalValueGenerator(const tChar* __info);

    /*! default destructor */
    virtual ~cSignalValueGenerator();

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
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

    /*! This Function is always called when any property has changed. This should be the only place
    *    to read from the properties itself and store their values in a member.
    *    \param [in] strName the name of the property that has changed.
    *    \return   Returns a standard result code.
    */
    tResult PropertyChanged(const tChar* strName);

protected: // Implement cBaseQtFilter

    /*! Creates the widget instance
    *   \return handle to view
    */
    tHandle CreateView();

    /*! Destroys the widget instance
    *   \result Returns a standard result code.
    */
    tResult ReleaseView();

    /*! the output pin to send the value for speed controller */
    cOutputPin     m_oOutputValue1;

    /*! the output pin to send the value for steering controller */
    cOutputPin     m_oOutputValue2;

    /*! descriptor for actuator values data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutput;
    /*! the id for the f32value of the media description for the pins */
    tBufferID m_szIDOutputF32Value;
    /*! the id for the arduino time stamp of the media description for the pins */
    tBufferID m_szIDOutputArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsOutputSet;

    /*! descriptor for actuator values data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutput2;
    /*! the id for the f32value of the media description for the pins */
    tBufferID m_szIDOutput2F32Value;
    /*! the id for the arduino time stamp of the media description for the pins */
    tBufferID m_szIDOutput2ArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsOutput2Set;

    /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;


private:

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;
    /*! if set to true the outputs are generated from the list otherwise the default values are send*/
    tBool m_bEnableOutput;

    /*! the default output for value 1 */
    tFloat32 m_f32DefaultValue1;

    /*! the default output for value 2 */
    tFloat32 m_f32DefaultValue2;

    /*! holds the timestamp when the list was started*/
    tFloat32 m_f32StartTimeStamp;

    /*! the array with the timestamps and their corresponding signal values */
    vector<tValueStruct> m_signalValues;

    /*! gets the current value from the vector for the given timestamp
    * \param timestamp the current timestamp of which the value has to be get
    * \param valueID the id of which the value has to be get (only 1 or 2 are valid by now: columns in vector)
    * \return current value corresponding to timestamp
    */
    tFloat32 getCurrentValue(tFloat32 timestamp, tInt8 valueID);

    /*! gets the default value for the given id
    * \param valueID the id of which the value has to be get (only 1 or 2 are valid by now: columns in vector)
    * \return default value
    */
    tFloat32 getDefaultValue(tInt8 valueID);

public slots:

    /*! signal for receiving the tuples of the gui
    * \param stateFlag current state
    * \param time timestamp in milliseconds
    * \param value1 the value 1
    * \param value2 the value 2
    */
    void OnSendTuple(int stateFlag, float time, float value1, float value2);


};

#endif /** @} */ // end of group
