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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/
#include "stdafx.h"
#include "cBoolValueGenerator.h"


ADTF_FILTER_PLUGIN("AADC Bool Value Generator", __guid, cBoolValueGenerator);




cBoolValueGenerator::cBoolValueGenerator(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info)
{
}

cBoolValueGenerator::~cBoolValueGenerator()
{
}

tHandle cBoolValueGenerator::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);

    // make the qt connections
    connect(m_pWidget->m_btSendValueFalse, SIGNAL(clicked()), this, SLOT(OnTransmitValueFalse()));
    connect(m_pWidget->m_btSendValueTrue, SIGNAL(clicked()), this, SLOT(OnTransmitValueTrue()));

    return (tHandle)m_pWidget;
}

tResult cBoolValueGenerator::ReleaseView()
{
    // delete the widget if present
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cBoolValueGenerator::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        //get the media description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
        //get description
        tChar const * strDescignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");

        // checks if exists
        RETURN_IF_POINTER_NULL(strDescignalValue);

        //get mediatype
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //get mediatype description
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));

        //create pin for output
        RETURN_IF_FAILED(m_oBoolValuePin.Create("BoolValue",pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oBoolValuePin));

        RETURN_NOERROR;
    }
    else if(eStage == StageGraphReady)
    {
        // media descriptions ids not set by now
        m_bIDsBoolValueOutput = tFalse;
    }
    RETURN_NOERROR;
}

tResult cBoolValueGenerator::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));

    RETURN_NOERROR;
}

tResult cBoolValueGenerator::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult cBoolValueGenerator::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult cBoolValueGenerator::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}


void cBoolValueGenerator::OnTransmitValueFalse()
{
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    tBool bValue = tFalse;
    tUInt32 ui32TimeStamp = 0;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        // set the id if not already done
        if(!m_bIDsBoolValueOutput)
        {
            pCoderOutput->GetID("bValue", m_szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDArduinoTimestampOutput);
            m_bIDsBoolValueOutput = tTrue;
        }

        // set value from sample
        pCoderOutput->Set(m_szIDBoolValueOutput, (tVoid*)&bValue);
        pCoderOutput->Set(m_szIDArduinoTimestampOutput, (tVoid*)&(ui32TimeStamp));
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    //transmit media sample over output pin
    m_oBoolValuePin.Transmit(pMediaSample);
}

void cBoolValueGenerator::OnTransmitValueTrue()
{
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    tBool bValue = tTrue;
    tUInt32 ui32TimeStamp = 0;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        // set the id if not already done
        if(!m_bIDsBoolValueOutput)
        {
            pCoderOutput->GetID("bValue", m_szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDArduinoTimestampOutput);
            m_bIDsBoolValueOutput = tTrue;
        }

        // set value from sample
        pCoderOutput->Set(m_szIDBoolValueOutput, (tVoid*)&bValue);
        pCoderOutput->Set(m_szIDArduinoTimestampOutput, (tVoid*)&(ui32TimeStamp));
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    //transmit media sample over output pin
    m_oBoolValuePin.Transmit(pMediaSample);
}
