/*****************************************************************************
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

#ifndef EXTERNPYTHONSERVER_CLASS_H
#define EXTERNPYTHONSERVER_CLASS_H

#include "stdafx.h"

// these files are first available after the install script was performed once
#include "ExtService.h"
#include "ExtIf_types.h"
#include "user_classification_structs.h"


#define OID_ADTF_FILTER_DEF "adtf.aadc.ExtPythonServer" //unique for a filter
#define ADTF_FILTER_DESC "AADC External Python Server"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "ExtPythonServer"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "this filter creates the THRIFT RPC Client which can receive data and transmit image data. \n$Rev:: 62947"


//necessary because stupid WinGDI.h redefines it
#undef GetObject


/*!
* @defgroup ExtPythonServer External Python Server
* @{
* \image html ExternalPythonServer.PNG "Plugin ExtPythonServer"
*
* This filter transmits images over a thrift RPC to a Remote Thrift RPC Server. This server could be implemented in Python, Java etc and receives the image data and can perform some operations with the data. For further information refer to
* https://thrift.apache.org/
* and
* https://thrift.apache.org/tutorial/py
*
* The following diagramm shows the data flow of the ADTF External Python Server Filter and a corresponding Python Instance. An Example for this python module is also included in the AADC source
* \image html ExternalPythonServerGraph.png "Data Flow ExtPythonServer"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li Boost   v.1.57.0
* \li Thrift  v.0.9.3
* \li OpenSSL  v.1.0.1
* \li OpenCV  v.3.2.0
*
* <b> Plugin Properties</b> \n
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Thrift Port<td>Port number for Thrift Server<td>1833
* <tr><td>Thrift IP Address<td>The ip v4 adress of the thrift RPC server<td>127.0.0.1
* <tr><td>Enable debug outputs to console <td>If enabled additional debug information is printed to the console (Warning: decreases performance)<td>false
* <tr><td>Send Bitmap Format<td>If a complete bitmap including the bitmap header should be send to thrift server<td>false
* </table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_Input<td>pin for imagedata to transmit to thrift<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Response<td>response string from python server<td>MEDIA_TYPE_STRUCTURED_DATA<td>MEDIA_SUBTYPE_STRUCT_STRUCTURED
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_ExtPythonServer
* <tr><td>Filename<td>aadc_ExtPythonServer.plb
* <tr><td>Version<td>1.0.0
* </table>
*/

//!  Class of Extern Thrift RPC Client
/*!
* This class is the main class of the Extern Thrift RPC Client Filter
*/
class ExtPythonServer : public adtf::cFilter,
    public adtf::IKernelThreadFunc
{
public:
	cCriticalSection CritOut;
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Tool,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );

    /*! decides wether or not the debug output are printed in the ADTF console */
    static const cString   PropEnableConsoleLogName;
    /*! description of the property */
    static const cString   PropEnableConsoleLogDesc;
    /*! default value of the property */
    static const tBool    PropEnableConsoleLogDefault;

    /*! decides wether or not the debug output are printed in the ADTF console */
    static const cString   PropSendBitmapFormatName;
    /*! description of the property */
    static const cString   PropSendBitmapFormatDesc;
    /*! default value of the property */
    static const tInt    PropSendBitmapFormatDefault;

    /*! name of the property */
    static const cString PropThriftPortName;
    /*! description of the property */
    static const cString PropThriftPortDesc;
    /*! default value of the property */
    static const tInt PropThriftPortDefault;

    /*! name of the property */
    static const cString PropThriftIPV4AddressName;
    /*! description of the property */
    static const cString PropThriftIPV4AddressDesc;
    /*! default value of the property */
    static const cString PropThriftIPV4AddressDefault;

public:
    /*! constructor for template class
    *    \param __info   [in] This is the name of the filter instance.
    */
    ExtPythonServer(const tChar* __info);

    /*! the destructor for this class
    */
    ~ExtPythonServer();

    /*! Implements the default cFilter state machine call. It will be
    *    called automatically by changing the filters state and needs
    *    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*! This Function is always called when any property has changed. This should be the only place
    *    to read from the properties itself and store their values in a member.
    *
    *    \param [in] strName the name of the property that has changed.
    *    \
    *    \return   Returns a standard result code.
    */
    tResult PropertyChanged(const tChar* strName);

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *    \param [in] pSource Pointer to the sending pin's IPin interface.
    *    \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *    \param [in] nParam1 Optional integer parameter.
    *    \param [in] nParam2 Optional integer parameter.
    *    \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *    \
    *    \return   Returns a standard result code.
    *    \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *    You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*!  Implements the default cFilter state machine calls. It will be
    *     called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *     and can be overwritten by the special filter.
    *
    *     \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *     If not using the cException smart pointer, the interface has to
    *     be released by calling Unref().
    *
    *     \return Standard Result Code.
    *
    *     \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *     (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*! Implements the default cFilter state machine call. It will be
    *     called automatically by changing the filters state and needs
    *     to be overwritten by the special filter.
    *     Please see  page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *     \param  eStage [in]    The Init function will be called when the filter state changes as follows:\n
    *     \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *                        If not using the cException smart pointer, the interface has to
    *                        be released by calling Unref().
    *
    *     \return Standard Result Code.
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);



private:


    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
        \param pFormat the new format for the input and input pin
        \result RETURN_NOERROR if successful
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! processes a received media sample with UTC time
     *  \param pMediaSample the incomining the media sample
     *  \result Returns a standard result code.
     *
        */
    tResult Process(IMediaSample* pMediaSample);

    /*! transmits the received string response via the output pin
     * \param response the response string from thrift server
     *  \result Returns a standard result code.
     */
    tResult Transmit(const std::vector<cnnClassificationResult>& response);

    /*! input pin for utc time */
    cVideoPin      m_inputPinImageData;

    /*! input pin for utc time */
    cOutputPin      m_outputResponseData;

    /*!  bitmap format of input pin */
    tBitmapFormat m_sInputFormat;

    /*! the opencv data type */
    tInt m_openCVType;


    /*! the struct with all the properties*/
    struct filterProperties
    {
        /*! stores if debug output should be printed */
        tBool   enableDebugOutput;
        /*! port number for server*/
        tUInt32  server_port;
        /*! the ip v4 adress of the thrift RPC server */
        cString server_addressIPV4;
        /*! Uf a complete bitmap including the bitmap header should be send to thrift server */
        tInt sendFormat;
    }
    /*! the filter properties of this class */
    m_filterProperties;

    /*! the external interface thrift client */
    boost::shared_ptr<ext_iface::ExtServiceClient> m_thriftClient;

    /*! critical section for send to thrift */
    cCriticalSection    m_critSectionSendToPython;

    /*! the thread for the server */
    cKernelThread           m_oThriftClientThread;


protected:
    /*! The ThreadFunc callback.
	 * \param pThread    [in] The kernel thread called from.
	 * \param pvUserData [in] The user data of the thread, given at
	 *                        @see cKernelThread::Create.
	 * \param szUserData [in] The user data size of the thread, given at @see
	 *                        cKernelThread::Create.
	 * \return Standard result
	 */
    tResult ThreadFunc(cKernelThread* pThread, tVoid* pvUserData, tSize szUserData);

    /*! buffer for received data from inputPin and send to thrift */
    ext_iface::TDataRaw m_thriftRawMessageBuffer;

    /*! buffer for received data from inputPin and send to thrift */
    ext_iface::TImageParams m_thriftImageParamsBuffer;


};

/*!
*@}
*/

#endif //ExtPythonServer_CLASS_H
