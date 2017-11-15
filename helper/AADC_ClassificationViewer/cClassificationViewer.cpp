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
* $Author:: spiesra $  $Date:: 2017-05-11 14:55:07#$ $Rev:: 63067   $
**********************************************************************/
#include "stdafx.h"
#include "cClassificationViewer.h"
#include "user_classification_structs.h"

ADTF_FILTER_PLUGIN("AADC Classification Viewer", __guid, cClassificationViewer);


cClassificationViewer::cClassificationViewer(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info)
{
}

cClassificationViewer::~cClassificationViewer()
{
}

tHandle cClassificationViewer::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);

    connect(this, SIGNAL(sendClassificationResult(QString, double, QString, QString)), m_pWidget, SLOT(OnNewClassificationResult(QString, double, QString, QString)));
    connect(this, SIGNAL(resetClassificationResults()), m_pWidget, SLOT(OnResetResults()));

    return (tHandle)m_pWidget;
}

tResult cClassificationViewer::ReleaseView()
{
    // delete the widget if present
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cClassificationViewer::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {

        //create pin for output
        RETURN_IF_FAILED(m_oClassificationPin.Create("classification", new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oClassificationPin));

        RETURN_NOERROR;
    }
    else if (eStage == StageGraphReady)
    {
    }
    RETURN_NOERROR;
}

tResult cClassificationViewer::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &m_oClassificationPin)
        {
            std::vector<cnnClassificationResult> classificationResults;
            classificationResults.resize(pMediaSample->GetSize() / sizeof(cnnClassificationResult));
            //get the date from the media sample
            tVoid* pIncomingData;
            if (IS_OK(pMediaSample->Lock((const tVoid**)&pIncomingData)))
            {
                //make copy
                memcpy(classificationResults.data(), pIncomingData, pMediaSample->GetSize());
                pMediaSample->Unlock(pIncomingData);
            }
            //send the results to the ui
            if (!classificationResults.empty())
            {
                emit resetClassificationResults();
                for (std::vector<cnnClassificationResult>::iterator it = classificationResults.begin(); it != classificationResults.end(); it++)
                {
                    emit sendClassificationResult(it->cnnClassificationDesc, it->probability, it->orientation, it->kid ? "True" : "False");
				}
			}
		}
	}
	else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
	}
	RETURN_NOERROR;
}


