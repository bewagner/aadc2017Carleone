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
* $Author:: spiesra $  $Date:: 2017-05-09 07:12:04#$ $Rev:: 62946   $
**********************************************************************/

#ifndef _TEMPLATE_PROJECT_DATA_H_
#define _TEMPLATE_PROJECT_DATA_H_

// Our example data consists of a single tFloat64 value
typedef tFloat64 tTemplateData;

/*!
 * Major-type of our template data type.
 *
 * In order to be able to distinguish our data we define custom media type identifiers.
 * Please note that in this example we do not require any additional metadata, so we use 
 * an "ordinary" OID_ADTF_MEDIA_TYPE media type with the following major- and subtype. If you want to pass on
 * additional meta data please have a look at the demo_versioned_media_type example.
 */ 
#define MEDIA_TYPE_TEMPLATE     0x00040677  

/*!
 * Sub-type of our template data type.
 */
#define MEDIA_SUBTYPE_TEMPLATE  0x00040677  

#endif
