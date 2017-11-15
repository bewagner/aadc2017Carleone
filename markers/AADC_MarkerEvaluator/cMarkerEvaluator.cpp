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
#include "aadc_roadSign_enums.h"
#include "cMarkerEvaluator.h"
#include "displaywidget.h"


ADTF_FILTER_PLUGIN("AADC Road Sign Evaluator", __guid, cMarkerEvaluator);


cMarkerEvaluator::cMarkerEvaluator(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info)
{
    SetPropertyBool("Debug Output to Console", tFalse);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)");
}

cMarkerEvaluator::~cMarkerEvaluator()
{
}

tHandle cMarkerEvaluator::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);

    return (tHandle)m_pWidget;
}

tResult cMarkerEvaluator::ReleaseView()
{
    // delete widget if it exists
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cMarkerEvaluator::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)
    {
        RETURN_IF_FAILED(CreateInputPins());
    }
    else if (eStage == StageNormal)
    {
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if(eStage == StageGraphReady)
    {
        // no ids were set so far
        m_bIDsRoadSignSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult cMarkerEvaluator::Start(__exception)
{
    return cBaseQtFilter::Start(__exception_ptr);
}

tResult cMarkerEvaluator::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult cMarkerEvaluator::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult cMarkerEvaluator::CreateInputPins()
{
    RETURN_IF_FAILED(m_InputPin.Create("Road_Sign", new cMediaType(0, 0, 0, "tRoadSign"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_InputPin));
    RETURN_NOERROR;
}


tResult cMarkerEvaluator::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pMediaSample != NULL && m_pDescriptionInput != NULL)
        {
            tInt16 i16ID = 0;
            tFloat32 f32Area = 0;

            {
                // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionInput,pMediaSample,pCoderInput);

                // get IDs
                if(!m_bIDsRoadSignSet)
                {
                    pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
                    pCoderInput->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
                    m_bIDsRoadSignSet = tTrue;
                }

                // get the values from sample
                pCoderInput->Get(m_szIDRoadSignI16Identifier, (tVoid*)&i16ID);
                pCoderInput->Get(m_szIDRoadSignF32Imagesize, (tVoid*)&f32Area);

            }

            if (m_bDebugModeEnabled)   LOG_INFO(cString::Format("Evaluator: Id -> %d Area: %f", i16ID, f32Area));

            // push data to gui
            m_pWidget->pushData(i16ID);
        }
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && pSource != NULL)
    {
        cObjectPtr<IMediaType> pType;
        pSource->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));

            m_pDescriptionInput = pMediaTypeDesc;
        }
    }
    RETURN_NOERROR;
}







