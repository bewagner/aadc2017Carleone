/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hartlan  $  $Date:: 2017-05-12 13:09:20#$ $Rev:: 63137   $
**********************************************************************/

#include "stdafx.h"
#include "cSensorVisualization.h"
#include "widget.h"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, cSensorVisualization);

cSensorVisualization::cSensorVisualization(const tChar* __info) : cBaseQtFilter(__info)
{

}

cSensorVisualization::~cSensorVisualization()
{

}

tResult cSensorVisualization::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageGraphReady)
    {
    }
    RETURN_NOERROR;
}

tResult cSensorVisualization::Start(__exception)
{
    return cBaseQtFilter::Start(__exception_ptr);
}

tResult cSensorVisualization::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult cSensorVisualization::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tHandle cSensorVisualization::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new Widget(pWidget);
    return (tHandle)m_pWidget;
}

tResult cSensorVisualization::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cSensorVisualization::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;

    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(strUltrasonicStruct);
    cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
    RETURN_IF_POINTER_NULL(strDescWheelData);
    cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strVoltageStruct = pDescManager->GetMediaDescription("tVoltageStruct");
    RETURN_IF_POINTER_NULL(strVoltageStruct);
    cObjectPtr<IMediaType> pTypeVoltageStruct = new cMediaType(0, 0, 0, "tVoltageStruct", strVoltageStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData));
    RETURN_IF_FAILED(pTypeVoltageStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionVoltData));
    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));
    RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelLeftData));
    RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelRightData));

    RETURN_IF_FAILED(m_oInputUssStruct.Create("UltrasonicStruct", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputUssStruct));

    RETURN_IF_FAILED(m_oInputInerMeasUnit.Create("InerMeasUnitStruct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputInerMeasUnit));

    RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

    RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

    RETURN_IF_FAILED(m_oInputVoltageStruct.Create("VoltageStruct", pTypeVoltageStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputVoltageStruct));

    RETURN_NOERROR;
}
tResult cSensorVisualization::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oInputUssStruct)
        {
            ProcessUSSSample(pMediaSample);
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

tResult cSensorVisualization::ProcessVoltageSample(IMediaSample* pMediaSample)
{
    if (pMediaSample->GetSize() == sizeof(tVoltageStruct))
    {
        tVoltageStruct* pSampleData = NULL;
        if (IS_OK(pMediaSample->Lock((const tVoid**)&pSampleData)))
        {
            //set the voltages of the GUI
            m_pWidget->setVoltageActuator(pSampleData->tActuatorVoltage.f32Value);
            m_pWidget->setVoltageActuatorCell1(pSampleData->tActuatorCell1.f32Value);
            m_pWidget->setVoltageActuatorCell2(pSampleData->tActuatorCell2.f32Value);
            m_pWidget->setVoltageSensors(pSampleData->tSensorVoltage.f32Value);
            m_pWidget->setVoltageSensorsCell1(pSampleData->tSensorCell1.f32Value);
            m_pWidget->setVoltageSensorsCell2(pSampleData->tSensorCell2.f32Value);
            m_pWidget->setVoltageSensorsCell3(pSampleData->tSensorCell3.f32Value);
            m_pWidget->setVoltageSensorsCell4(pSampleData->tSensorCell4.f32Value);
            m_pWidget->setVoltageSensorsCell5(pSampleData->tSensorCell5.f32Value);
            m_pWidget->setVoltageSensorsCell6(pSampleData->tSensorCell6.f32Value);
            pMediaSample->Unlock(pSampleData);
        }
    }
    RETURN_NOERROR;
}


tResult cSensorVisualization::ProcessUSSSample(IMediaSample* pMediaSample)
{
    if (pMediaSample->GetSize() == sizeof(tUltrasonicStruct))
    {
        tUltrasonicStruct* pSampleData = NULL;
        if (IS_OK(pMediaSample->Lock((const tVoid**)&pSampleData)))
        {
            //send all the the data to the GUI
            m_pWidget->setUsFL(pSampleData->tFrontLeft.f32Value);

            m_pWidget->setUsFCL(pSampleData->tFrontCenterLeft.f32Value);

            m_pWidget->setUsFC(pSampleData->tFrontCenter.f32Value);

            m_pWidget->setUsFCR(pSampleData->tFrontCenterRight.f32Value);

            m_pWidget->setUsFR(pSampleData->tFrontRight.f32Value);

            m_pWidget->setUsSL(pSampleData->tSideLeft.f32Value);

            m_pWidget->setUsSR(pSampleData->tSideRight.f32Value);

            m_pWidget->setUsRCL(pSampleData->tRearLeft.f32Value);

            m_pWidget->setUsRC(pSampleData->tRearCenter.f32Value);

            m_pWidget->setUsRCR(pSampleData->tRearRight.f32Value);

            pMediaSample->Unlock(pSampleData);
        }
    }

    RETURN_NOERROR;
}

tResult cSensorVisualization::ProcessInerMeasUnitSample(IMediaSample* pMediaSample)
{
    tFloat32 f32A_x = 0;
    tFloat32 f32A_y = 0;
    tFloat32 f32A_z = 0;

    tFloat32 f32G_x = 0;
    tFloat32 f32G_y = 0;
    tFloat32 f32G_z = 0;

    tFloat32 f32M_x = 0;
    tFloat32 f32M_y = 0;
    tFloat32 f32M_z = 0;

    tFloat32 f32roll = 0;
    tFloat32 f32pitch = 0;
    tFloat32 f32yaw = 0;

    static bool hasID = false;
    static tBufferID szIDInerMeasUnitArduinoTimestamp;
    static tBufferID szIDInerMeasUnitF32A_x;
    static tBufferID szIDInerMeasUnitF32A_y;
    static tBufferID szIDInerMeasUnitF32A_z;

    static tBufferID szIDInerMeasUnitF32G_x;
    static tBufferID szIDInerMeasUnitF32G_y;
    static tBufferID szIDInerMeasUnitF32G_z;

    static tBufferID szIDInerMeasUnitF32M_x;
    static tBufferID szIDInerMeasUnitF32M_y;
    static tBufferID szIDInerMeasUnitF32M_z;

    static tBufferID szIDInerMeasUnitF32roll;
    static tBufferID szIDInerMeasUnitF32pitch;
    static tBufferID szIDInerMeasUnitF32yaw;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("ui32ArduinoTimestamp", szIDInerMeasUnitArduinoTimestamp);

            pCoderInput->GetID("f32A_x", szIDInerMeasUnitF32A_x);
            pCoderInput->GetID("f32A_y", szIDInerMeasUnitF32A_y);
            pCoderInput->GetID("f32A_z", szIDInerMeasUnitF32A_z);

            pCoderInput->GetID("f32G_x", szIDInerMeasUnitF32G_x);
            pCoderInput->GetID("f32G_y", szIDInerMeasUnitF32G_y);
            pCoderInput->GetID("f32G_z", szIDInerMeasUnitF32G_z);

            pCoderInput->GetID("f32M_x", szIDInerMeasUnitF32M_x);
            pCoderInput->GetID("f32M_y", szIDInerMeasUnitF32M_y);
            pCoderInput->GetID("f32M_z", szIDInerMeasUnitF32M_z);

            pCoderInput->GetID("f32roll", szIDInerMeasUnitF32roll);
            pCoderInput->GetID("f32pitch", szIDInerMeasUnitF32pitch);
            pCoderInput->GetID("f32yaw", szIDInerMeasUnitF32yaw);


            hasID = true;
        }

        pCoderInput->Get(szIDInerMeasUnitF32A_x, (tVoid*)&f32A_x);
        pCoderInput->Get(szIDInerMeasUnitF32A_y, (tVoid*)&f32A_y);
        pCoderInput->Get(szIDInerMeasUnitF32A_z, (tVoid*)&f32A_z);

        pCoderInput->Get(szIDInerMeasUnitF32G_x, (tVoid*)&f32G_x);
        pCoderInput->Get(szIDInerMeasUnitF32G_y, (tVoid*)&f32G_y);
        pCoderInput->Get(szIDInerMeasUnitF32G_z, (tVoid*)&f32G_z);

        pCoderInput->Get(szIDInerMeasUnitF32M_x, (tVoid*)&f32M_x);
        pCoderInput->Get(szIDInerMeasUnitF32M_y, (tVoid*)&f32M_y);
        pCoderInput->Get(szIDInerMeasUnitF32M_z, (tVoid*)&f32M_z);

        pCoderInput->Get(szIDInerMeasUnitF32roll, (tVoid*)&f32roll);
        pCoderInput->Get(szIDInerMeasUnitF32pitch, (tVoid*)&f32pitch);
        pCoderInput->Get(szIDInerMeasUnitF32yaw, (tVoid*)&f32yaw);

        m_pWidget->setImu(f32A_x, f32A_y, f32A_z, f32G_x, f32G_y, f32G_z, f32M_x, f32M_y, f32M_z, f32roll, f32pitch, f32yaw);


    }
    RETURN_NOERROR;
}

tResult cSensorVisualization::ProcessWheelSampleLeft(IMediaSample* pMediaSample)
{
    tUInt32 ui32Tach = 0;
    tInt8 i8Direction = 0;

    static bool hasID = false;
    static tBufferID szIDWheelDataI8WheelDir;
    static tBufferID szIDWheelDataUi32WheelTach;
    static tBufferID szIDWheelDataArduinoTimestamp;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelLeftData, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("i8WheelDir", szIDWheelDataI8WheelDir);
            pCoderInput->GetID("ui32WheelTach", szIDWheelDataUi32WheelTach);
            pCoderInput->GetID("ui32ArduinoTimestamp", szIDWheelDataArduinoTimestamp);
            hasID = tTrue;
        }

        pCoderInput->Get(szIDWheelDataUi32WheelTach, (tVoid*)&ui32Tach);
        pCoderInput->Get(szIDWheelDataI8WheelDir, (tVoid*)&i8Direction);

        m_pWidget->setWheelLeft(ui32Tach, i8Direction);


    }
    RETURN_NOERROR;
}



tResult cSensorVisualization::ProcessWheelSampleRight(IMediaSample* pMediaSample)
{
    tUInt32 ui32Tach = 0;
    tInt8 i8Direction = 0;

    static bool hasID = false;
    static tBufferID szIDWheelDataI8WheelDir;
    static tBufferID szIDWheelDataUi32WheelTach;
    static tBufferID szIDWheelDataArduinoTimestamp;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelRightData, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("i8WheelDir", szIDWheelDataI8WheelDir);
            pCoderInput->GetID("ui32WheelTach", szIDWheelDataUi32WheelTach);
            pCoderInput->GetID("ui32ArduinoTimestamp", szIDWheelDataArduinoTimestamp);
            hasID = tTrue;
        }

        pCoderInput->Get(szIDWheelDataUi32WheelTach, (tVoid*)&ui32Tach);
        pCoderInput->Get(szIDWheelDataI8WheelDir, (tVoid*)&i8Direction);

        m_pWidget->setWheelRight(ui32Tach, i8Direction);

    }
    RETURN_NOERROR;
}
