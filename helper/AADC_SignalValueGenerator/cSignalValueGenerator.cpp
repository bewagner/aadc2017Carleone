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
#include "cSignalValueGenerator.h"


ADTF_FILTER_PLUGIN("AADC Signal Value Generator", __guid, cSignalValueGenerator);




cSignalValueGenerator::cSignalValueGenerator(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info),
    m_bEnableOutput(tFalse)
{
    SetPropertyInt("Actuator Update Rate [Hz]",30);
    SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)");
    SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0);
    SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

    SetPropertyFloat("Default Value 1",0);
    SetPropertyStr("Default Value 1" NSSUBPROP_DESCRIPTION, "Defines the default value which is transmitted when nothing is read from the list");
    SetPropertyBool("Default Value 1" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("Default Value 2",0);
    SetPropertyStr("Default Value 2" NSSUBPROP_DESCRIPTION, "Defines the default value which is transmitted when nothing is read from the list");
    SetPropertyBool("Default Value 2" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyStr("File for Default Values","");
    SetPropertyBool("File for Default Values" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("File for Default Values" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "txt Files (*.txt)");
    SetPropertyStr("File for Default Values" NSSUBPROP_DESCRIPTION, "Set the file with the default files here");
}

cSignalValueGenerator::~cSignalValueGenerator()
{
}

tHandle cSignalValueGenerator::CreateView()
{
    //Get path of file with default values
    cFilename fileConfig = GetPropertyStr("File for Default Values");
    if ( fileConfig.IsEmpty() )
    {
        LOG_ERROR("Entry for File for Default Values missing");
        fileConfig = "Default.txt";
    }
    //create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");
    //check if marker configuration file exits
    if (!(cFileSystem::Exists(fileConfig)))
    {
        LOG_ERROR("File for Default Values missing");
        fileConfig = "Default.txt";
    }
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(fileConfig.GetPtr(), pWidget);

    connect(m_pWidget, SIGNAL(sendTuple(int, float,float,float)), this, SLOT(OnSendTuple(int, float,float,float)));

    return (tHandle)m_pWidget;
}

tResult cSignalValueGenerator::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cSignalValueGenerator::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)
    {

        //get the description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        //get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


        //get mediatype description for output data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));

        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput2));

        //create pin for speed output data
        RETURN_IF_FAILED(m_oOutputValue1.Create("Value_1", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputValue1));

        //create pin for steering output data
        RETURN_IF_FAILED(m_oOutputValue2.Create("Value_2", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputValue2));

        RETURN_NOERROR;
    }
    else if(eStage == StageNormal)
    {
        m_f32DefaultValue1 = static_cast<tFloat32>(GetPropertyFloat("Default Value 1"));
        m_f32DefaultValue2 = static_cast<tFloat32>(GetPropertyFloat("Default Value 2"));
    }
    else if(eStage == StageGraphReady)
    {
        // no ids were set so far
        m_bIDsOutputSet = tFalse;
        m_bIDsOutput2Set = tFalse;
    }
    RETURN_NOERROR;
}


tResult cSignalValueGenerator::PropertyChanged(const tChar* strName)
{
    //read from property if it was changed
    if (cString::IsEqual("Default Value 1", strName))
    {
        m_f32DefaultValue1 = static_cast<tFloat32>(GetPropertyFloat("Default Value 1"));
    }
    else if (cString::IsEqual("Default Value 2", strName))
    {
        m_f32DefaultValue2 = static_cast<tFloat32>(GetPropertyFloat("Default Value 2"));
    }

    RETURN_NOERROR;
}
tResult cSignalValueGenerator::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));

    //clears the vector and disables output
    m_bEnableOutput = tFalse;
    m_signalValues.clear();

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
                                          NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}

tResult cSignalValueGenerator::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    if (nActivationCode == IRunnable::RUN_TIMER)
    {
        // actuator timer was called, time to transmit actuator samples
        if (pvUserData==&m_hTimerOutput && m_bEnableOutput==tTrue)
        {

            //create new media sample for steering controller
            {
                cObjectPtr<IMediaSample> pMediaSampleValue1;
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue1));

                //allocate memory with the size given by the descriptor
                cObjectPtr<IMediaSerializer> pSerializer1;
                m_pDescriptionOutput->GetMediaSampleSerializer(&pSerializer1);
                tInt nSize = pSerializer1->GetDeserializedSize();
                RETURN_IF_FAILED(pMediaSampleValue1->AllocBuffer(nSize));
                tFloat32 value1 = getCurrentValue(_clock->GetStreamTime(),1);
                {
                    // focus for sample write lock
                    //write date to the media sample with the coder of the descriptor
                    __adtf_sample_write_lock_mediadescription(m_pDescriptionOutput,pMediaSampleValue1,pCoder);
                    // get the IDs for the items in the media sample
                    if(!m_bIDsOutputSet)
                    {
                        pCoder->GetID("f32Value", m_szIDOutputF32Value);
                        pCoder->GetID("ui32ArduinoTimestamp", m_szIDOutputArduinoTimestamp);
                        m_bIDsOutputSet = tTrue;
                    }
                    pCoder->Set(m_szIDOutputF32Value, (tVoid*)&value1);
                }

                //transmit media sample over output pin

                RETURN_IF_FAILED(pMediaSampleValue1->SetTime(_clock->GetStreamTime()));


                RETURN_IF_FAILED(m_oOutputValue1.Transmit(pMediaSampleValue1));
            }
            //create new media sample for speed controller
            {
                cObjectPtr<IMediaSample> pMediaSampleValue2;
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue2));

                //allocate memory with the size given by the descriptor
                cObjectPtr<IMediaSerializer> pSerializerSpeed;
                m_pDescriptionOutput2->GetMediaSampleSerializer(&pSerializerSpeed);
                tInt nSizeSpeed = pSerializerSpeed->GetDeserializedSize();
                RETURN_IF_FAILED(pMediaSampleValue2->AllocBuffer(nSizeSpeed));

                tFloat32 value2 = getCurrentValue(_clock->GetStreamTime(),2);

                {
                    // focus for sample write lock
                    //write date to the media sample with the coder of the descriptor
                    __adtf_sample_write_lock_mediadescription(m_pDescriptionOutput2,pMediaSampleValue2,pCoder2);

                    // get the IDs for the items in the media sample
                    if(!m_bIDsOutput2Set)
                    {
                        pCoder2->GetID("f32Value", m_szIDOutput2F32Value);
                        pCoder2->GetID("ui32ArduinoTimestamp", m_szIDOutput2ArduinoTimestamp);
                        m_bIDsOutput2Set = tTrue;
                    }

                    pCoder2->Set(m_szIDOutput2F32Value, (tVoid*)&value2);
                }

                //transmit media sample over output pin
                RETURN_IF_FAILED(pMediaSampleValue2->SetTime(_clock->GetStreamTime()));
                RETURN_IF_FAILED(m_oOutputValue2.Transmit(pMediaSampleValue2));
            }
        }

    }
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}


tResult cSignalValueGenerator::Stop(__exception)
{
    if (m_hTimerOutput)
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    //clears the vector and disables output
    m_bEnableOutput = tFalse;
    m_signalValues.clear();

    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult cSignalValueGenerator::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}


void cSignalValueGenerator::OnSendTuple(int stateFlag, float time, float value1, float value2)
{
    // append signal to list
    if (stateFlag ==0) m_signalValues.clear();

    if (m_signalValues.empty())
    {
        tValueStruct newStruct;
        newStruct.timeStamp= static_cast<tFloat32>(time);
        newStruct.f32Value1 = static_cast<tFloat32>(value1);
        newStruct.f32Value2 = static_cast<tFloat32>(value2);

        m_signalValues.push_back(newStruct);
        LOG_INFO(cString::Format("received %f, %f, %f",m_signalValues.back().timeStamp,m_signalValues.back().f32Value1,m_signalValues.back().f32Value2));
    }
    else if (time > (m_signalValues.back().timeStamp))
    {
        tValueStruct newStruct;
        newStruct.timeStamp= static_cast<tFloat32>(time);
        newStruct.f32Value1 = static_cast<tFloat32>(value1);
        newStruct.f32Value2 = static_cast<tFloat32>(value2);

        m_signalValues.push_back(newStruct);
        LOG_INFO(cString::Format("received %f, %f, %f",(m_signalValues.back().timeStamp),(m_signalValues.back().f32Value1),(m_signalValues.back().f32Value2)));
    }
    else
        LOG_WARNING("Entry could not be inserted in list because timestamp is smaller than the last one");

    if (stateFlag ==2)
    {
        m_f32StartTimeStamp = _clock->GetStreamTime();
        m_bEnableOutput = tTrue;
    }
}

tFloat32 cSignalValueGenerator::getDefaultValue(tInt8 valueID)
{
    switch (valueID)
    {
    case 1:
        return m_f32DefaultValue1;
        break;
    case 2:
        return m_f32DefaultValue2;
        break;
    default:
        return m_f32DefaultValue1;
        break;
    }
}

tFloat32 cSignalValueGenerator::getCurrentValue(tFloat32 timestamp, tInt8 valueID)
{
    //get time difference in milliseconds
    timestamp = (_clock->GetStreamTime()-m_f32StartTimeStamp)/1000;

    // list empty or output disabled, return default
    if (m_signalValues.empty() || m_bEnableOutput== tFalse) return getDefaultValue(valueID);

    // timestamp greater than last value , return default
    if (timestamp > (m_signalValues.back().timeStamp) ) return getDefaultValue(valueID);

    vector<tValueStruct>::iterator it2, it1;

    // it2 is bigger then timestamp, it1 is smaller than timestamp, find it2:
    for (it2 = m_signalValues.begin(); it2<m_signalValues.end(); it2++)
        if ((*it2).timeStamp>timestamp) break;
    // the timestamp is smaller than first timestamp in list so we have to calculate from zero
    if (it2 == m_signalValues.begin())
    {
        switch (valueID)
        {
        case 1:
            return getDefaultValue(valueID)+((*it2).f32Value1-getDefaultValue(valueID))/((*it2).timeStamp)*(timestamp);
            break;
        case 2:
            return getDefaultValue(valueID)+((*it2).f32Value2-getDefaultValue(valueID))/((*it2).timeStamp)*(timestamp);
            break;
        }
    }

    //set it2  to the element before it2:
    it1 = it2;
    --it1;
    switch (valueID)
    {
    case 1:
        return  (*it1).f32Value1+ ((*it2).f32Value1-(*it1).f32Value1)/((*it2).timeStamp-(*it1).timeStamp)*(timestamp -(*it1).timeStamp);
        break;
    case 2:
        return (*it1).f32Value2+ ((*it2).f32Value2-(*it1).f32Value2)/((*it2).timeStamp-(*it1).timeStamp)*(timestamp -(*it1).timeStamp);
        break;
    }

    //should never happen (only if valueID is not 1 or 2:
    return m_f32DefaultValue1;
}
