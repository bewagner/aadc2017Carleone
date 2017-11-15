/*********************************************************************
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

#ifndef _USER_CLASSIFICATION_STRUCTS_
#define _USER_CLASSIFICATION_STRUCTS_



/*! Encapsulates the result of a classification. */
struct classificationResult
{
	/*! The classification description */
	tChar classificationDesc[128];
    
	/*! The probability */
	tFloat64 probability;

	/*! Default constructor. */
	classificationResult()
		{
			memset(classificationDesc, 0, sizeof(classificationDesc));
			probability = 0;
		}

	/*!
	 * Constructor.
	 *
	 * \param   classString The class description string
	 * \param   prob        The probability of the result
	 */
	classificationResult(std::string classString, double prob)
		{
        
			if (classString.size() > 128)
			{
				memcpy(classificationDesc, classString.data(), 128);
			}
			else
			{
				memset(classificationDesc, 0, sizeof(classificationDesc));
				memcpy(classificationDesc, classString.data(), classString.size());
			}
			probability = prob;
		}

};

struct cnnClassificationResult
{
    /*! The classification description */
    tChar cnnClassificationDesc[128];
	
	/*! The probability */
	tFloat64 probability;

	/*! The border points of the detections bounding box*/
	tFloat64 ymin;
	tFloat64 xmin;
	tFloat64 ymax;
	tFloat64 xmax;
	
    /*! The orientation of a person */
    tChar orientation[128];

	/*! Wether this entity is a kid */
	tBool kid;
	
	
	/*! Default constructor. */
	cnnClassificationResult()
		{
			memset(cnnClassificationDesc, 0, sizeof(cnnClassificationDesc));
			memset(orientation, 0, sizeof(orientation));
			probability = 0;
			ymin = 0;
			xmin = 0;
			ymax = 0;
			xmax = 0;
			kid =  tFalse;
			
		}
	/*!
	 * Constructor.
	 *
	 * \param   classString The class description string
	 * \param   prob        The probability of the result
	 * \param   ymin        The minimal y point of the bounding box
	 * \param   xmin        The minimal x point of the bounding box
	 * \param   ymax        The maximal y point of the bounding box
	 * \param   xmax        The maximal x point of the bounding box
	 */
	cnnClassificationResult(std::string classString, double prob, double yminInput, double xminInput,  double ymaxInput,  double xmaxInput, std::string orientationString,  bool kidInput)
		{
        
			if (classString.size() > 128)
			{
				memcpy(cnnClassificationDesc, classString.data(), 128);
			}
			else
			{
				memset(cnnClassificationDesc, 0, sizeof(cnnClassificationDesc));
				memcpy(cnnClassificationDesc, classString.data(), classString.size());
			}

			if (orientationString.size() > 128)
			{
				memcpy(orientation, orientationString.data(), 128);
			}
			else
			{
				memset(orientation, 0, sizeof(orientation));
				memcpy(orientation, orientationString.data(), orientationString.size());
			}
			probability = prob;
			ymin = yminInput;
			xmin = xminInput;
			ymax = ymaxInput;
			xmax = xmaxInput;
			kid =  kidInput;
		}
};
#endif // _USER_CLASSIFICATION_STRUCTS_
