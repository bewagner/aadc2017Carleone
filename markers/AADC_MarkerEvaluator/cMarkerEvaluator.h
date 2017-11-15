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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/
#ifndef cMarkerEvaluatorFilter_H
#define cMarkerEvaluatorFilter_H

#include "stdafx.h"
#include "displaywidget.h"
#define __guid "adtf.aadc.markerEvaluator"

/*! @defgroup MarkerEvaluator Marker Evaluator
*  @{
*
* The results of the marker detection filter can be visualized with this tool. The input pin Road_Sign has to be connected with the output pin Road_Sign of the Marker Detection Filter.
* The GUI shows the symbol of the road sign with the highest frequency in the incoming samples. On the right side a graph is plotted which shows the frequency of all signs, i.e. Marker IDs, in the incoming Media Samples from the marker detection filter.
*
*  \image html MarkerEvaluator.PNG "Plugin Marker Evaluator"
*
* \b Dependencies \n
* This plugin needs the following libraries:
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Debug Output to Console<td>If enabled additional debug information is printed to the console (Warning: decreases performance)<td>false
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>Road_Sign<td>road sign from marker detector<td>tRoadSign
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/markers/AADC_MarkerEvaluator
* <tr><td>Filename<td>aadc_markerEvaluator.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*!
* This is the main class of the Marker Evaluator Filter
*/
class cMarkerEvaluator : public QObject, public cBaseQtFilter
{
    /*! sets the filter version */
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Marker Evaluator", adtf::OBJCAT_Tool, "AADC Marker Evaluator", 1, 0, 0, "");

    Q_OBJECT

public:
    /*! default constructor of class
    \param __info info pointer
    */
    cMarkerEvaluator(const tChar* __info);
    /*! default destructor */
    virtual ~cMarkerEvaluator();

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Mess		*    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    */
    tResult Init(tInitStage eStage, ucom::IException**  __exception_ptr);

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
    *     called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *     and can be overwritten by the special filter.
    *     \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *     If not using the cException smart pointer, the interface has to
    *     be released by calling Unref().
    *     \return Standard Result Code.
    *     \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *     (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*! Implements the default cFilter state machine call. It will be
    *     called automatically by changing the filters state and needs
    *     to be overwritten by the special filter.
    *     Please see  page_filter_life_cycle for further information on when the state of a filter changes.
    *     \param  eStage [in]    The Init function will be called when the filter state changes as follows:\n
    *     \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *                        If not using the cException smart pointer, the interface has to
    *                        be released by calling Unref().
    *
    *     \return Standard Result Code.
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *    \param [in] pSource Pointer to the sending pin's IPin interface.
    *    \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *    \param [in] nParam1 Optional integer parameter.
    *    \param [in] nParam2 Optional integer parameter.
    *    \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *    \return   Returns a standard result code.
    *    \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *    You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:
    /*! the input pin for the struct from the detector */
    cInputPin m_InputPin;

    /*! the descriptor for the input pin*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInput;
    /*! the id for the i16Identifier of the media description for output pin */
    tBufferID m_szIDRoadSignI16Identifier;
    /*! the id for the f32Imagesize of the media description for output pin */
    tBufferID m_szIDRoadSignF32Imagesize;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsRoadSignSet;

    /*! the gui of this filter */
    DisplayWidget* m_pWidget;

    /*! this function creates the input pins
    *    \return   Returns a standard result code.
    */
    tResult CreateInputPins();

    /*! Creates the widget instance
    *   \return handle to view
    */
    tHandle CreateView();

    /*! Destroys the widget instance
    *   \result Returns a standard result code.
    */
    tResult ReleaseView();

    /*! indicates wheter information is printed to the console or not */
    tBool m_bDebugModeEnabled;
};

/*! @} */ // end of group


#endif//cMarkerEvaluatorFilter_H

