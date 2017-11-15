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



#ifndef _BOOL_VALUE_HEADER
#define _BOOL_VALUE_HEADER

#define __guid "adtf.aadc.classificationViewer"

#include "stdafx.h"
#include "displaywidget.h"

/*! @defgroup cClassificationViewer Classification Viewer
*  @{
*
* With this plugin the results of the External Python Server plugin can be visualized.
*
*  \image html ClassificationViewer.PNG "Plugin Classification Viewer"
*
* \b Dependencies \n
* This plugin needs the following libraries:

*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType<th>Details
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubTyp
* <tr><td>classification<td>results from the classifier<td>MEDIA_TYPE_STRUCTURED_DATA<td>MEDIA_SUBTYPE_STRUCT_STRUCTURED<td>
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/helper/AADC_ClassificationViewer
* <tr><td>Filename<td>aadc_classificationViewer.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! this ist the main class of the plugin classification viewer */
class cClassificationViewer : public QObject, public cBaseQtFilter
{

    /*! set the filter ID and the filter version */
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Classification Viewer", OBJCAT_Auxiliary, "Classification Viewer", 1, 0, 0, "");

    Q_OBJECT


public:

    /*! default constructor
    * \param __info info argument for filter
    */
    cClassificationViewer(const tChar* __info);

    /*! default destructor */
    virtual ~cClassificationViewer();

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

private:

    /*! The displayed widget */
    DisplayWidget *m_pWidget;

    /*! input pin for the run command*/
    cInputPin        m_oClassificationPin;

signals:
    /*! send the classification result to the GUI
    * \param results string with result
    * \param probability probability of result
    */
    void sendClassificationResult(QString results, double probability, QString orientation,  QString kid);

    /*! resets the classification results in the GUI */
    void resetClassificationResults();

};

#endif /** @} */ // end of group
