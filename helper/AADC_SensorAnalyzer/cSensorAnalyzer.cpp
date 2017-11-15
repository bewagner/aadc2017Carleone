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
* $Author:: spiesra $  $Date:: 2017-05-18 16:51:13#$ $Rev:: 63512   $
**********************************************************************/

#include "stdafx.h"
#include "cSensorAnalyzer.h"
#include "aadc_structs.h"

ADTF_FILTER_PLUGIN("AADC Sensor Analyzer", __guid, cSensorAnalyzer);


cSensorAnalyzer::cSensorAnalyzer(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info)
{
    SetPropertyStr("Directory for Sensorpresets","");
    SetPropertyBool("Directory for Sensorpresets" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool("Directory for Sensorpresets" NSSUBPROP_DIRECTORY, tTrue);
    SetPropertyStr("Directory for Sensorpresets" NSSUBPROP_DESCRIPTION, "Here you have to select the folder which contains the sensor preset files");
}


cSensorAnalyzer::~cSensorAnalyzer()
{
}

tHandle cSensorAnalyzer::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);

    connect(this, SIGNAL(SensorDataChanged(int,float)), m_pWidget, SLOT(SetSensorData(int,float)));
    connect(this, SIGNAL(DirectoryReceived(QString)), m_pWidget, SLOT(SetDirectory(QString)));
    connect(m_pWidget->GetDropDownWidget(), SIGNAL(currentIndexChanged(const QString&)), this, SLOT(SetConfiguration(const QString&)));

    return (tHandle)m_pWidget;
}

tResult cSensorAnalyzer::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cSensorAnalyzer::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        THROW_IF_FAILED(CreateInputPins(__exception_ptr));
    }
    else if (StageNormal == eStage)
    {
        m_sensorPresets.resize(LIST_LENGTH);
        //THROW_IF_FAILED(LoadConfigurationData("default"));
    }
    else if(eStage == StageGraphReady)
    {
        //m_pWidget->SetSensorPresets(m_sensorPresets);
        cFilename filepath = GetPropertyStr("Directory for Sensorpresets");
        ADTF_GET_CONFIG_FILENAME(filepath);
        cString path = filepath.CreateAbsolutePath(".");
        emit DirectoryReceived(QString(path.GetPtr()));

        // no ids were set so far
        m_bIDsVoltageSet= tFalse;
        m_bIDsInerMeasUnitSet= tFalse;
        m_bIDsWheelDataLeftSet= tFalse;
        m_bIDsWheelDataRightSet = tFalse;
        m_bIDsUltrasonicSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult cSensorAnalyzer::CreateInputPins(__exception)
{
    //get the description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    //get description for sensor data pins
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    //get mediatype for ultrasonic sensor data pins
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get description for inertial measurement sensor data pin
    tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);

    //get mediatype for ultrasonic sensor data pins
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get description for wheel sensors data pins
    tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
    RETURN_IF_POINTER_NULL(strDescWheelData);
    //get mediatype for wheeldata sensor data pins
    cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    tChar const * strVoltageStruct = pDescManager->GetMediaDescription("tVoltageStruct");
    RETURN_IF_POINTER_NULL(strVoltageStruct);
    cObjectPtr<IMediaType> pTypeVoltageStruct = new cMediaType(0, 0, 0, "tVoltageStruct", strVoltageStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(strUltrasonicStruct);
    cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get mediatype description for ultrasonic sensor data type
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData));

    //get mediatype description for voltage sensor data type
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionVoltData));

    //get mediatype description for inertial measurement unit sensor data type
    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));

    //get mediatype description for wheel sensor data type
    RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataLeft));
    RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataRight));

    //get mediatype description for wheel sensor data type
    RETURN_IF_FAILED(pTypeVoltageStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionVoltData));

    //create pins for ultrasonic sensor data
    RETURN_IF_FAILED(m_oInputUssStruct.Create("UltrasonicStruct", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputUssStruct));

    //create pins for voltage sensor data
    RETURN_IF_FAILED(m_oInputVoltageStruct.Create("VoltageStruct", pTypeVoltageStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputVoltageStruct));

    //create pin for inertial measurement unit data
    RETURN_IF_FAILED(m_oInputInerMeasUnit.Create("InerMeasUnitStruct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputInerMeasUnit));

    //create pin for wheel left sensor data
    RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

    //create pin for wheel right data
    RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));
    RETURN_NOERROR;
}

tResult cSensorAnalyzer::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));
    THROW_IF_POINTER_NULL(_kernel);


    //create the timer for the transmitting actuator values
    RETURN_NOERROR;
}

tResult cSensorAnalyzer::Stop(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult cSensorAnalyzer::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult cSensorAnalyzer::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oInputUssStruct)
        {
            RETURN_IF_FAILED(ProcessUSSSample(pMediaSample));
        }
        else if (pSource == &m_oInputInerMeasUnit)
        {
            RETURN_IF_FAILED(ProcessInerMeasUnitSample(pMediaSample));
        }
        else if (pSource == &m_oInputWheelLeft)
        {
            RETURN_IF_FAILED(ProcessWheelSampleLeft(pMediaSample));
        }
        else if (pSource == &m_oInputWheelRight)
        {
            RETURN_IF_FAILED(ProcessWheelSampleRight(pMediaSample));
        }
        else if (pSource == &m_oInputVoltageStruct)
        {
            RETURN_IF_FAILED(ProcessVoltageSample(pMediaSample));
        }
    }
    RETURN_NOERROR;
}


tResult cSensorAnalyzer::ProcessVoltageSample(IMediaSample* pMediaSample)
{
    if (pMediaSample->GetSize() == sizeof(tVoltageStruct))
    {
        tVoltageStruct* pSampleData = NULL;
        if (IS_OK(pMediaSample->Lock((const tVoid**)&pSampleData)))
        {
            //set the voltages of the GUI
            emit SensorDataChanged(LIST_VOLTAGE_MEASUREMENT, static_cast<float>(pSampleData->tSensorVoltage.f32Value));
            emit SensorDataChanged(LIST_VOLTAGE_SPEEDCTR, static_cast<float>(pSampleData->tActuatorVoltage.f32Value));

            pMediaSample->Unlock(pSampleData);
        }
    }
    RETURN_NOERROR;
}


tResult cSensorAnalyzer::ProcessUSSSample(IMediaSample* pMediaSample)
{
    if (pMediaSample->GetSize() == sizeof(tUltrasonicStruct))
    {
        tUltrasonicStruct* pSampleData = NULL;
        if (IS_OK(pMediaSample->Lock((const tVoid**)&pSampleData)))
        {
            //send all the the data to the GUI
            emit SensorDataChanged(LIST_ULTRASONIC_FRONT_LEFT, static_cast<float>(pSampleData->tFrontLeft.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_FRONT_CENTER_LEFT, static_cast<float>(pSampleData->tFrontCenterLeft.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_FRONT_CENTER, static_cast<float>(pSampleData->tFrontCenter.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_FRONT_CENTER_RIGHT, static_cast<float>(pSampleData->tFrontCenterRight.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_FRONT_RIGHT, static_cast<float>(pSampleData->tFrontRight.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_SIDE_LEFT, static_cast<float>(pSampleData->tSideLeft.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_SIDE_RIGHT, static_cast<float>(pSampleData->tSideRight.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_REAR_LEFT, static_cast<float>(pSampleData->tRearLeft.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_REAR_CENTER, static_cast<float>(pSampleData->tRearCenter.f32Value));
            emit SensorDataChanged(LIST_ULTRASONIC_REAR_RIGHT, static_cast<float>(pSampleData->tRearRight.f32Value));

            pMediaSample->Unlock(pSampleData);
        }
    }

    RETURN_NOERROR;
}

tResult cSensorAnalyzer::ProcessInerMeasUnitSample(IMediaSample* pMediaSample)
{
    //write values with zero
    tInerMeasUnitData imuData;

    {
        // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData,pMediaSample,pCoderInput);

        // get the IDs for the items in the media sample
        if(!m_bIDsInerMeasUnitSet)
        {
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDInerMeasUnitArduinoTimestamp);

            pCoderInput->GetID("f32A_x", m_szIDInerMeasUnitF32A_x);
            pCoderInput->GetID("f32A_y", m_szIDInerMeasUnitF32A_y);
            pCoderInput->GetID("f32A_z", m_szIDInerMeasUnitF32A_z);
                                        
            pCoderInput->GetID("f32G_x", m_szIDInerMeasUnitF32G_x);
            pCoderInput->GetID("f32G_y", m_szIDInerMeasUnitF32G_y);
            pCoderInput->GetID("f32G_z", m_szIDInerMeasUnitF32G_z);
                                         
            pCoderInput->GetID("f32M_x", m_szIDInerMeasUnitF32M_x);
            pCoderInput->GetID("f32M_y", m_szIDInerMeasUnitF32M_y);
            pCoderInput->GetID("f32M_z", m_szIDInerMeasUnitF32M_z);

            pCoderInput->GetID("f32roll", m_szIDInerMeasUnitF32roll);
            pCoderInput->GetID("f32pitch", m_szIDInerMeasUnitF32pitch);
            pCoderInput->GetID("f32yaw", m_szIDInerMeasUnitF32yaw);
            m_bIDsInerMeasUnitSet = tTrue;
        }

        //get values from media sample
        pCoderInput->Get(m_szIDInerMeasUnitF32A_x, (tVoid*)&imuData.f32A_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_y, (tVoid*)&imuData.f32A_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_z, (tVoid*)&imuData.f32A_z);
                         
        pCoderInput->Get(m_szIDInerMeasUnitF32G_x, (tVoid*)&imuData.f32G_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32G_y, (tVoid*)&imuData.f32G_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32G_z, (tVoid*)&imuData.f32G_z);
                         
        pCoderInput->Get(m_szIDInerMeasUnitF32M_x, (tVoid*)&imuData.f32M_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32M_y, (tVoid*)&imuData.f32M_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32M_z, (tVoid*)&imuData.f32M_z);
                         
        pCoderInput->Get(m_szIDInerMeasUnitF32roll, (tVoid*)&imuData.f32Roll);
        pCoderInput->Get(m_szIDInerMeasUnitF32pitch, (tVoid*)&imuData.f32Pitch);
        pCoderInput->Get(m_szIDInerMeasUnitF32yaw, (tVoid*)&imuData.f32Yaw);

        
        // multiplicate with 180/pi (*180/M_PI) to get angle in degree
        emit SensorDataChanged(LIST_GYROSCOPE_X_ACC, static_cast<tFloat>(imuData.f32A_x));
        emit SensorDataChanged(LIST_GYROSCOPE_Y_ACC, static_cast<tFloat>(imuData.f32A_y));
        emit SensorDataChanged(LIST_GYROSCOPE_Z_ACC, static_cast<tFloat>(imuData.f32A_z));
        emit SensorDataChanged(LIST_GYROSCOPE_X_ROT, static_cast<tFloat>(imuData.f32G_x));
        emit SensorDataChanged(LIST_GYROSCOPE_Y_ROT, static_cast<tFloat>(imuData.f32G_y));
        emit SensorDataChanged(LIST_GYROSCOPE_Z_ROT, static_cast<tFloat>(imuData.f32G_z));
        emit SensorDataChanged(LIST_GYROSCOPE_X_MAG, static_cast<tFloat>(imuData.f32M_x));
        emit SensorDataChanged(LIST_GYROSCOPE_Y_MAG, static_cast<tFloat>(imuData.f32M_y));
        emit SensorDataChanged(LIST_GYROSCOPE_Z_MAG, static_cast<tFloat>(imuData.f32M_z));
        emit SensorDataChanged(LIST_GYROSCOPE_YAW, static_cast<tFloat>(imuData.f32Yaw));
        emit SensorDataChanged(LIST_GYROSCOPE_PITCH, static_cast<tFloat>(imuData.f32Pitch));
        emit SensorDataChanged(LIST_GYROSCOPE_ROLL, static_cast<tFloat>(imuData.f32Roll));

    }
    RETURN_NOERROR;
}

tResult cSensorAnalyzer::ProcessWheelSampleLeft(IMediaSample* pMediaSample)
{
    __synchronized_obj(m_oProcessWheelDataCritSection);
    //write values with zero
    tUInt32 ui32Tach = 0;
    tInt8 i8Direction = 0;
    {
        // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelDataLeft,pMediaSample,pCoderInput);

        // get the IDs for the items in the media sample
        if(!m_bIDsWheelDataLeftSet)
        {
            pCoderInput->GetID("i8WheelDir", m_szIDWheelDataLeftI8WheelDir);
            pCoderInput->GetID("ui32WheelTach", m_szIDWheelDataLeftUi32WheelTach);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDWheelDataLeftArduinoTimestamp);
            m_bIDsWheelDataLeftSet = tTrue;
        }

        //get values from media sample
        pCoderInput->Get(m_szIDWheelDataLeftUi32WheelTach, (tVoid*)&ui32Tach);
        pCoderInput->Get(m_szIDWheelDataLeftI8WheelDir, (tVoid*)&i8Direction);

        //LOG_INFO(cString::Format("test %f, %d",ui32Tach,i8Direction));
        //emit signal to gui
        // 
        emit SensorDataChanged(LIST_WHEEL_TACH_LEFT, static_cast<tFloat>(ui32Tach));
        emit SensorDataChanged(LIST_WHEEL_DIR_LEFT, static_cast<tFloat>(i8Direction));       
    }
    RETURN_NOERROR;
}


tResult cSensorAnalyzer::ProcessWheelSampleRight(IMediaSample* pMediaSample)
{
    __synchronized_obj(m_oProcessWheelDataCritSection);
    //write values with zero
    tUInt32 ui32Tach = 0;
    tInt8 i8Direction = 0;
    {
        // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelDataRight, pMediaSample, pCoderInput);

        // get the IDs for the items in the media sample
        if (!m_bIDsWheelDataRightSet)
        {
            pCoderInput->GetID("i8WheelDir", m_szIDWheelDataRightI8WheelDir);
            pCoderInput->GetID("ui32WheelTach", m_szIDWheelDataRightUi32WheelTach);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDWheelDataRightArduinoTimestamp);
            m_bIDsWheelDataRightSet = tTrue;
        }

        //get values from media sample
        pCoderInput->Get(m_szIDWheelDataRightUi32WheelTach, (tVoid*)&ui32Tach);
        pCoderInput->Get(m_szIDWheelDataRightI8WheelDir, (tVoid*)&i8Direction);

        //LOG_INFO(cString::Format("test %f, %d",ui32Tach,i8Direction));
        //emit signal to gui
        // 
        emit SensorDataChanged(LIST_WHEEL_TACH_RIGHT, static_cast<tFloat>(ui32Tach));
        emit SensorDataChanged(LIST_WHEEL_DIR_RIGHT, static_cast<tFloat>(i8Direction));
    }
    RETURN_NOERROR;
}


tResult cSensorAnalyzer::LoadConfigurationData(cString filename)
{
    //Get path of configuration file
    //EDS macro is resolved automatically because it is a file property
    m_fileConfig = filename;//GetPropertyStr("Filename for Sensorpresets");

    if (m_fileConfig.IsEmpty())
        LOG_WARNING("Configuration file not found for Analyzer");

    ADTF_GET_CONFIG_FILENAME(m_fileConfig);
    m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        cDOMElementRefList oElems;
        if(IS_OK(oDOM.FindNodes("presets/sensorPreset", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                cDOMElement* pConfigElement;
                tSensorPreset newSensorPreset;
                if (IS_OK((*itElem)->FindNode("sensor", pConfigElement)))
                {
                    newSensorPreset.sensorName = cString(pConfigElement->GetData());
                    if (IS_OK((*itElem)->FindNode("nominalValue", pConfigElement)))
                        newSensorPreset.nominalValue = static_cast<tFloat32>(cString(pConfigElement->GetData()).AsFloat64());
                    if (IS_OK((*itElem)->FindNode("maxPosDeviation", pConfigElement)))
                        newSensorPreset.maxPosDeviation = static_cast<tFloat32>(cString(pConfigElement->GetData()).AsFloat64());
                    if (IS_OK((*itElem)->FindNode("maxNegDeviation", pConfigElement)))
                        newSensorPreset.maxNegDeviation = static_cast<tFloat32>(cString(pConfigElement->GetData()).AsFloat64());
                }
                /*LOG_INFO(cString::Format("Name %s, Nominal: %f, NegDev: %f, PosDev: %f",
                newSensorPreset.sensorName.GetPtr(),
                newSensorPreset.nominalValue,
                newSensorPreset.maxNegDeviation,
                newSensorPreset.maxPosDeviation
                ));*/
                addParsedElement(newSensorPreset);
            }
        }
        else
        {
            LOG_WARNING("Configured configuration file does not contain valid xml scheme");
        }
    }
    else
    {
        LOG_WARNING("Configured configuration file not found (yet).Will appear after extracting the extended data");
    }

    RETURN_NOERROR;
}

tResult cSensorAnalyzer::addParsedElement(tSensorPreset newSensorStruct)
{
    if (newSensorStruct.sensorName.Compare("VOLTAGE_MEASUREMENT")==0)
        m_sensorPresets.at(LIST_VOLTAGE_MEASUREMENT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("VOLTAGE_SPEEDCTR")==0)
        m_sensorPresets.at(LIST_VOLTAGE_SPEEDCTR)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_FRONT_LEFT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_FRONT_LEFT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_FRONT_CENTER_LEFT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_FRONT_CENTER_LEFT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_FRONT_CENTER")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_FRONT_CENTER)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_FRONT_CENTER_RIGHT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_FRONT_CENTER_RIGHT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_FRONT_RIGHT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_FRONT_RIGHT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_SIDE_LEFT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_SIDE_LEFT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_SIDE_RIGHT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_SIDE_RIGHT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_REAR_LEFT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_REAR_LEFT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_REAR_CENTER")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_REAR_CENTER)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("ULTRASONIC_REAR_RIGHT")==0)
        m_sensorPresets.at(LIST_ULTRASONIC_REAR_RIGHT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_X_ACC")==0)
        m_sensorPresets.at(LIST_GYROSCOPE_X_ACC)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_Y_ACC")==0)
        m_sensorPresets.at(LIST_GYROSCOPE_Y_ACC)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_Z_ACC")==0)
        m_sensorPresets.at(LIST_GYROSCOPE_Z_ACC)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_X_ROT") == 0)
        m_sensorPresets.at(LIST_GYROSCOPE_X_ROT) = newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_Y_ROT") == 0)
        m_sensorPresets.at(LIST_GYROSCOPE_Y_ROT) = newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_Z_ROT") == 0)
        m_sensorPresets.at(LIST_GYROSCOPE_Z_ROT) = newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_X_MAG") == 0)
        m_sensorPresets.at(LIST_GYROSCOPE_X_MAG) = newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_Y_MAG") == 0)
        m_sensorPresets.at(LIST_GYROSCOPE_Y_MAG) = newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_Z_MAG") == 0)
        m_sensorPresets.at(LIST_GYROSCOPE_Z_MAG) = newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_YAW")==0)
        m_sensorPresets.at(LIST_GYROSCOPE_YAW)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_PITCH")==0)
        m_sensorPresets.at(LIST_GYROSCOPE_PITCH)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("GYROSCOPE_ROLL")==0)
        m_sensorPresets.at(LIST_GYROSCOPE_ROLL)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("WHEEL_TACH_RIGHT")==0)
        m_sensorPresets.at(LIST_WHEEL_TACH_RIGHT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("WHEEL_TACH_LEFT")==0)
        m_sensorPresets.at(LIST_WHEEL_TACH_LEFT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("WHEEL_DIR_RIGHT")==0)
        m_sensorPresets.at(LIST_WHEEL_DIR_RIGHT)=newSensorStruct;
    else if (newSensorStruct.sensorName.Compare("WHEEL_DIR_LEFT")==0)
        m_sensorPresets.at(LIST_WHEEL_DIR_LEFT)=newSensorStruct;

    RETURN_NOERROR;
}

void cSensorAnalyzer::SetConfiguration(const QString& filename)
{
    cFilename path = GetPropertyStr("Directory for Sensorpresets");
    path.Append("/");
    path.Append(filename.toStdString().c_str());
    LoadConfigurationData(path);
    m_pWidget->SetSensorPresets(m_sensorPresets);
}
