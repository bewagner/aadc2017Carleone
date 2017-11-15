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

#include "stdafx.h"
#include "cCarController.h"
#include "widget.h"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, cCarController);

cCarController::cCarController(const tChar* __info) : QObject(), cBaseQtFilter(__info)
{

}

cCarController::~cCarController()
{

}

tResult cCarController::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
    {
        CreateOutputPins();
    }
    else if (eStage == StageNormal)
    {
    }
    else if(eStage == StageGraphReady)
    {
    }
    RETURN_NOERROR;
}

tResult cCarController::Start(__exception)
{
    return cBaseQtFilter::Start(__exception_ptr);
}

tResult cCarController::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult cCarController::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult cCarController::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionFloat));
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));


    RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));


    RETURN_IF_FAILED(m_oOutputSteeringController.Create("SteeringController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputSteeringController));


    RETURN_IF_FAILED(m_oOutputHeadLight.Create("headLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputHeadLight));

    RETURN_IF_FAILED(m_oOutputBrakeLight.Create("brakeLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));

    RETURN_IF_FAILED(m_oOutputReverseLight.Create("reverseLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputReverseLight));

    RETURN_IF_FAILED(m_oOutputTurnLeft.Create("turnSignalLeftEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnLeft));

    RETURN_IF_FAILED(m_oOutputTurnRight.Create("turnSignalRightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnRight));

    RETURN_IF_FAILED(m_oOutputHazzard.Create("hazzardLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputHazzard));

    RETURN_IF_FAILED(m_oOutputUssEnableFront.Create("USSFrontEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUssEnableFront));

    RETURN_IF_FAILED(m_oOutputUssEnableRear.Create("USSRearEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUssEnableRear));
    ;

    RETURN_NOERROR;
}

tResult cCarController::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitBool);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDBoolValueOutput;
    static tBufferID szIDArduinoTimestampOutput;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        if(!hasID)
        {
            pCoderOutput->GetID("bValue", szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDBoolValueOutput, (tVoid*)&value);
        pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cCarController::TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitControl);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionFloat->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDValueOutput;
    static tBufferID szIDArduinoTimestampOutput;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionFloat, pMediaSample, pCoderOutput);

        if(!hasID)
        {
            pCoderOutput->GetID("f32Value", szIDValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDValueOutput, (tVoid*)&value);
        pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tHandle cCarController::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new Widget(pWidget);

    connect(m_pWidget, SIGNAL(steeringReceived(float)), this, SLOT(SendSteering(float)));
    connect(m_pWidget, SIGNAL(throttleReceived(float)), this, SLOT(SendThrottle(float)));
    connect(m_pWidget->getLightButtonGroup(), SIGNAL(buttonClicked(int)), this, SLOT(ToggleLights(int)));
    connect(m_pWidget, SIGNAL(buttonClicked(int)), this, SLOT(ToggleLights(int)));
    connect(m_pWidget->getUsButtonGroup(), SIGNAL(buttonClicked(int)), this, SLOT(ToggleUs(int)));

    return (tHandle)m_pWidget;
}

tResult cCarController::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

void cCarController::SendSteering(float value)
{
    TransmitFloatValue(&m_oOutputSteeringController, value, 0);
}

void cCarController::SendThrottle(float value)
{
    TransmitFloatValue(&m_oOutputSpeedController, value, 0);
}

void cCarController::ToggleLights(int buttonId)
{
    static bool headToggle;
    static bool reverseToggle;
    static bool brakeToggle;
    static bool turnRightToggle;
    static bool turnLeftToggle;
    static bool hazzardLightToggle;

    switch (buttonId)
    {
    case 0: // Head
        TransmitBoolValue(&m_oOutputHeadLight, !headToggle, 0);
        headToggle = !headToggle;
        LOG_INFO(cString::Format("Heads toggled: %d", headToggle));
        break;
    case 1: // Brake
        TransmitBoolValue(&m_oOutputBrakeLight, !brakeToggle, 0);
        brakeToggle = !brakeToggle;
        LOG_INFO(cString::Format("Brake toggled: %d", brakeToggle));
        break;
    case 2: // Reverse
        TransmitBoolValue(&m_oOutputReverseLight, !reverseToggle, 0);
        reverseToggle = !reverseToggle;
        LOG_INFO(cString::Format("Reverse toggled: %d", reverseToggle));
        break;
    case 3: // Hazard
        TransmitBoolValue(&m_oOutputHazzard, !hazzardLightToggle, 0);
        hazzardLightToggle = !hazzardLightToggle;
        LOG_INFO(cString::Format("Hazard toggled: %d", reverseToggle));
        break;
    case 4: // Left
        TransmitBoolValue(&m_oOutputTurnLeft, !turnLeftToggle, 0);
        turnLeftToggle = !turnLeftToggle;
        LOG_INFO(cString::Format("Turn Left toggled: %d", turnLeftToggle));
        break;
    case 5: // Right
        TransmitBoolValue(&m_oOutputTurnRight, !turnRightToggle, 0);
        turnRightToggle = !turnRightToggle;
        LOG_INFO(cString::Format("Turn right toggled: %d", turnRightToggle));
        break;

    default:
        break;
    }
}

void cCarController::ToggleUs(int buttonId)
{
    switch (buttonId)
    {
    case 0: // enable front
        TransmitBoolValue(&m_oOutputUssEnableFront, tTrue, 0);
        break;
    case 1: // disable front
        TransmitBoolValue(&m_oOutputUssEnableFront, tFalse, 0);
        break;
    case 2: // enable rear
        TransmitBoolValue(&m_oOutputUssEnableRear, tTrue, 0);
        break;
    case 3: // disable rear
        TransmitBoolValue(&m_oOutputUssEnableRear, tFalse, 0);
        break;

    default:
        break;
    }
}
