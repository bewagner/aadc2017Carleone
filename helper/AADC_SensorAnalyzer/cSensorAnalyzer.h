/*********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-17 14:13:43#$ $Rev:: 63363   $
**********************************************************************/



#ifndef _AADC_SENSORANALYZER
#define _AADC_SENSORANALYZER

#define __guid "adtf.aadc.sensorAnalyzer"

#include "stdafx.h"
#include "displaywidget.h"
#include "aadc_enums.h"


/*! @defgroup SensorAnalyzer Sensor Analyzer
*  @{
*
* With this filter the sensors can be checked by comparison of their values with a set of predefined parameters. The vehicle can be put in a special environment with measured distance to some obstacles around the car to check the ultrasonic sensors. The measured distance have to be written into in XML File and the Sensor Analyzer will verify the signal values. The GUI plots the box with the values within the specified range in green color, boxes with values out of the specified range in red color.
* The XML File contains presets for sensors by using the Sensor ID, setting a nominal value and maximum positive and negative deviation. The Sensor IDs can be found in the sample Sensorpreset file analyzer_presets.xml  in the folder configuration_files/Sensorpresets.
* In the Property Directory for Sensorpresets a folder can be specified where the filter should look for multiple preset files. The recognized files can be selected afterwards in the GUI with the drop down menu at the top.
*
*  \image html SensorAnalyzer.PNG "Plugin Sensor Analyzer"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li QT  v.4.7.1
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Directory for Sensorpresets<td>Here you have to select the folder which contains the sensor preset files<td>
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType<th>Details
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>UltrasonicStruct<td>tUltrasonicStruct
* <tr><td>VoltageStruct<td>tUltrasonicStruct
* <tr><td>InerMeasUnitStruct<td>tInerMeasUnitData
* <tr><td>WheelLeftStruct<td>tWheelData
* <tr><td>WheelRightStruct<td>tWheelData
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_SensorAnalyzer
* <tr><td>Filename<td>aadc_sensorAnalyzer.plb
* <tr><td>Version<td>1.1.0
* </table>
*
*/


/*! this is the main class of the sensor analyzer plugin */
class cSensorAnalyzer : public QObject, public cBaseQtFilter
{
    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Sensor Analyzer", OBJCAT_Application, "Sensor Analyzer", 1, 1, 0, "");

    Q_OBJECT

public: // construction

    /*! constructor for class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cSensorAnalyzer(const tChar * __info);


    /*! Destructor. */
    virtual ~cSensorAnalyzer();

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


protected: // Implement cBaseQtFilter

    /*! Creates the widget instance
    *   \return handle to view
    */
    tHandle CreateView();

    /*! Destroys the widget instance
    *   \result Returns a standard result code.
    */
    tResult ReleaseView();

    /*! this function creates all the input pins of filter
    * \param __exception_ptr exception pointer for error handling
    * \return standard error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

private:
    /*! Input pin for the ultrasonic front left data */
    cInputPin       m_oInputUssStruct;

    /*! Input pin for the voltage data of easurement circuit */
    cInputPin       m_oInputVoltageStruct;

    /*! Input pin for the inertial measurement unit data */
    cInputPin      m_oInputInerMeasUnit;

    /*! Input Pin for wheel struct*/
    cInputPin      m_oInputWheelLeft;

    /*! Input Pin for wheel struct*/
    cInputPin      m_oInputWheelRight;

    /*! descriptor for ultrasonic sensor data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData;
    /*! the id for the f32value of the media description for input pin of the ultrasoncic data */
    tBufferID m_szIDUltrasonicF32Value;
    /*! the id for the arduino time stamp of the media description for input pin of the ultrasoncic data */
    tBufferID m_szIDUltrasonicArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsUltrasonicSet;

    /*! descriptor for voltage sensor data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionVoltData;
    /*! the id for the f32value of the media description for input pin of the voltage data */
    tBufferID m_szIDVoltageF32Value;
    /*! the id for the arduino time stamp of the media description for input pin of the voltage data */
    tBufferID m_szIDVoltageArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsVoltageSet;

    /*! descriptor for intertial measurement unit sensor data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;

    /*! The size identifier iner meas unit f 32 an x coordinate */
    tBufferID m_szIDInerMeasUnitF32A_x;
    /*! The size identifier iner meas unit f 32 a y coordinate */
    tBufferID m_szIDInerMeasUnitF32A_y;
    /*! The size identifier iner meas unit f 32 a z coordinate */
    tBufferID m_szIDInerMeasUnitF32A_z;
    /*! The size identifier iner meas unit f 32 g x coordinate */
    tBufferID m_szIDInerMeasUnitF32G_x;
    /*! The size identifier iner meas unit f 32 g y coordinate */
    tBufferID m_szIDInerMeasUnitF32G_y;
    /*! The size identifier iner meas unit f 32 g z coordinate */
    tBufferID m_szIDInerMeasUnitF32G_z;
    /*! The size identifier iner meas unit f 32 m x coordinate */
    tBufferID m_szIDInerMeasUnitF32M_x;
    /*! The size identifier iner meas unit f 32 m y coordinate */
    tBufferID m_szIDInerMeasUnitF32M_y;
    /*! The size identifier iner meas unit f 32 m z coordinate */
    tBufferID m_szIDInerMeasUnitF32M_z;
    /*! The identifier iner meas unit f 32roll */
    tBufferID m_szIDInerMeasUnitF32roll;
    /*! The identifier iner meas unit f 32pitch */
    tBufferID m_szIDInerMeasUnitF32pitch;
    /*! The identifier iner meas unit f 32yaw */
    tBufferID m_szIDInerMeasUnitF32yaw;
    /*! the id for the arduino time stamp of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsInerMeasUnitSet;

    /*! descriptor for wheel sensor data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelDataRight;
    /*! the id for the ui32WheelTach of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataRightUi32WheelTach;
    /*! the id for the i8WheelDir of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataRightI8WheelDir;
    /*! the id for the arduino time stamp of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataRightArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsWheelDataRightSet;

    /*! descriptor for wheel sensor data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelDataLeft;
    /*! the id for the ui32WheelTach of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataLeftUi32WheelTach;
    /*! the id for the i8WheelDir of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataLeftI8WheelDir;
    /*! the id for the arduino time stamp of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataLeftArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsWheelDataLeftSet;

    /*! critical section for the processing of the ultrasonic data samples because function can be called from different onPinEvents */
    cCriticalSection    m_oProcessUsDataCritSection;

    /*! critical section for the processing of the voltage data samples because function can be called from different onPinEvents */
    cCriticalSection    m_oProcessVoltageDataCritSection;

    /*! critical section for the processing of the voltage data samples because function can be called from different onPinEvents */
    cCriticalSection    m_oProcessWheelDataCritSection;

    /*! the config file for the presets of the analyzer */
    cFilename m_fileConfig;

    /*! a list which holds the different sensor presets*/
    vector<tSensorPreset> m_sensorPresets;

private:
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

    /*! processes the mediasample with wheel data to the gui
    * \param pMediaSample the incoming mediasample
    * \return standard error code
    */
    tResult ProcessWheelSampleRight(IMediaSample* pMediaSample);

    /*! processes the mediasample with wheel data to the gui
    * \param pMediaSample the incoming mediasample
    * \return standard error code
    */
    tResult ProcessWheelSampleLeft(IMediaSample* pMediaSample);

    /*! processes the imu  data to the gui
    * \param pMediaSample the incoming mediasample
    * \return standard error code
    */
    tResult ProcessInerMeasUnitSample(IMediaSample* pMediaSample);

    /*! processes the steering sensor data to the gui
    * \param pMediaSample the incoming mediasample
    * \return standard error code
    */
    tResult ProcessSteeringSensSample(IMediaSample* pMediaSample);

    /*!
     * loads the configurationData for the nominal values.
     *
     * \param   filename    Filename of the file.
     *
     * \return standard error code
     */
    tResult LoadConfigurationData(cString filename);

    /*! adds the parsed element by LoadConfigurationData to m_sensorPresets
    * \param newSensorStruct the struct with the data to be added
    * \return standard error code
    */
    tResult addParsedElement(tSensorPreset newSensorStruct);

signals:
    /*! this signal sends the ultrasonic data to the gui widget
    * \param senorListId the enum which identifies the sensordata
    * \param value the value for the gui
    */
    void SensorDataChanged(int senorListId, float value);

    /*! this signal sends the the chosen dir path to the widget
    * \param filename the absolute path to the dir
    */
    void DirectoryReceived(QString filename);

public slots:
    /*! this slot sets the new loaded Configuration
    * \param filename the filename of the new configuration
    */
    void SetConfiguration(const QString& filename);
};

#endif /** @} */ // end of group
