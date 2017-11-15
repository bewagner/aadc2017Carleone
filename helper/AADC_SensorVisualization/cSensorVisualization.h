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
* $Author:: spie#$  $Date:: 2017-05-15 09:45:46#$ $Rev:: 63214   $
**********************************************************************/


#ifndef CSENSORVISUALIZATION_H
#define CSENSORVISUALIZATION_H

#include "stdafx.h"
class Widget;

#define OID_ADTF_FILTER_DEF                "adtf.aadc.SensorVisualization"
#define ADTF_FILTER_DESC                   "AADC Sensor Visualization"
#define ADTF_FILTER_VERSION_SUB_NAME       "Sensor Visualization"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL   "AADCSensorVisualization"
#define ADTF_FILTER_VERSION_STRING         "1.0.0"
#define ADTF_FILTER_VERSION_Major          1
#define ADTF_FILTER_VERSION_Minor          0
#define ADTF_FILTER_VERSION_Build          0
#define ADTF_FILTER_VERSION_LABEL "A filter to emplace the free space in the universe. \nCopyright (c)."
#define ADTF_CATEGORY OBJCAT_Application

/*! @defgroup SensorVisualization Sensor Visualization
*  @{
*
* Text here
*
*  \image html SensorVisualization.PNG "Plugin Sensor Visualization"
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
* <tr><th>Pin<th>Description<th>MajorType<th>SubType<th>Details*
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>UltrasonicStruct<td>tSignalValue
* <tr><td>InerMeasUnitStruct<td>tInerMeasUnitData
* <tr><td>WheelLeftStruct<td>tWheelData
* <tr><td>WheelRightStruct<td>tWheelData
* <tr><td>VoltageStruct<td>VoltageStruct
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_SensorVisualization
* <tr><td>Filename<td>aadc_SensorVisualization.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/

/*! the main class of the sensor visualization filter */
class cSensorVisualization : public cBaseQtFilter
{
    /*! set the filter version */
    ADTF_FILTER_VERSION(
        OID_ADTF_FILTER_DEF,
        ADTF_FILTER_DESC,
        ADTF_CATEGORY,
        ADTF_FILTER_VERSION_SUB_NAME,
        ADTF_FILTER_VERSION_Major,
        ADTF_FILTER_VERSION_Minor,
        ADTF_FILTER_VERSION_Build,
        ADTF_FILTER_VERSION_LABEL);

public:

    /*! constructor for template class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cSensorVisualization(const tChar* __info);

    /*! Destructor. */
    virtual ~cSensorVisualization();

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

    /*! Creates the widget instance
    *   \return handle to view
    */
    tHandle CreateView();

    /*! Destroys the widget instance
    *   \result Returns a standard result code.
    */
    tResult ReleaseView();

    /*! Theqt qt widget of this filter */
    Widget* m_pWidget;

    /*! creates all the input Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

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

    /*! input pin for uss data struct */
    cInputPin       m_oInputUssStruct;
    /*! input pin for voltage data struct */
    cInputPin       m_oInputVoltageStruct;
    /*! input pin for IMU data struct */
    cInputPin       m_oInputInerMeasUnit;
    /*! input pin for wheel left data struct */
    cInputPin       m_oInputWheelLeft;
    /*! input pin for wheel right data struct */
    cInputPin       m_oInputWheelRight;

    /*! mediadescription for ultrasonic data struct */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData;
    /*! mediadescription for ultrasonic data struct */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionVoltData;
    /*! mediadescription for ultrasonic data struct */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelRightData;
    /*! mediadescription for ultrasonic data struct */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelLeftData;
    /*! mediadescription for ultrasonic data struct */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;

    /*!
     * Process the wheel sample.
     *
     * \param [in,out]  pMediaSample    the media sample with wheel data right.
     *
     * \return  Returns a standard result code.
     */
    tResult ProcessWheelSampleRight(IMediaSample* pMediaSample);

    /*!
    * Process the wheel sample.
    *
    * \param [in,out]  pMediaSample    the mediaSample with wheel data left.
    *
    * \return  Returns a standard result code.
    */
    tResult ProcessWheelSampleLeft(IMediaSample* pMediaSample);

    /*!
     * Process the IMU sample contained in pMediaSample.
     *
     * \param [in,out]  pMediaSample    the mediaSample with IMU data
     *
     * \return  A tResult.
     */
    tResult ProcessInerMeasUnitSample(IMediaSample* pMediaSample);

    /*!
    * Process the voltage sample contained in pMediaSample.
    *
    * \param [in,out]  pMediaSample    the mediaSample with IMU data
    *
    * \return  A tResult.
    */
    tResult ProcessVoltageSample(IMediaSample* pMediaSample);

    /*!
    * Process the USS sample contained in pMediaSample.
    *
    * \param [in,out]  pMediaSample    the mediaSample with IMU data
    *
    * \return  A tResult.
    */
    tResult ProcessUSSSample(IMediaSample* pMediaSample);


private:

};

#endif /** @} */ // end of group
