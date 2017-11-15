/**
   Copyright (c)
   Audi Autonomous Driving Cup. All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
   4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   **********************************************************************
   * $Author:: spiesra $  $Date:: 2017-04-28 08:10:05#$ $Rev:: 62659   $
   **********************************************************************/
#include "stdafx.h"
#include "cClassificationVisualizer.h"
//#include "aadc_classification_structs.h"





// define the ADTF property names to avoid errors 
ADTF_FILTER_PLUGIN(ADTF_BFFT_FILTER_DESC,
				   OID_ADTF_BFFT_FILTER_DEF,
				   cClassificationVisualizer)



cClassificationVisualizer::cClassificationVisualizer(const tChar* __info) : cFilter(__info)
{
 SetPropertyBool("DEBUG::DebutOutput", tFalse); 
 SetPropertyStr("DEBUG::DebutOutput" NSSUBPROP_DESCRIPTION, "Debug-Output?"); 
 SetPropertyBool("DEBUG::DebutOutput" NSSUBPROP_ISCHANGEABLE, tTrue);    
	SetPropertyFloat("FILTERING::MinProbability", 75.0);
	SetPropertyStr("FILTERING::MinProbability" NSSUBPROP_DESCRIPTION, "The Minimum probability for a classified object to be displayed in the output picture.");
	SetPropertyBool("FILTERING::MinProbability" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("CROSSING::Left", 0.33);
	SetPropertyStr("CROSSING::Left" NSSUBPROP_DESCRIPTION, "End of the left part.");
	SetPropertyBool("CROSSING::Left" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("CROSSING::Right", 0.67);
	SetPropertyStr("CROSSING::Right" NSSUBPROP_DESCRIPTION, "begining of the right part.");
	SetPropertyBool("CROSSING::Right" NSSUBPROP_ISCHANGEABLE, tTrue);

	m_bFirstFrameDepthimage = true;
	
	SetPropertyFloat("CLASSIFICATION::LeftSidePercentage", 0.342);
	SetPropertyStr("CLASSIFICATION::LeftSidePercentage" NSSUBPROP_DESCRIPTION, "The percentage of the picture that is classified as left.");
	SetPropertyBool("CLASSIFICATION::LeftSidePercentage" NSSUBPROP_ISCHANGEABLE, tTrue);
	
	SetPropertyFloat("CLASSIFICATION::RightSidePercentage", 0.555);
	SetPropertyStr("CLASSIFICATION::RightSidePercentage" NSSUBPROP_DESCRIPTION, "The percentage of the picture that is classified as right.");
	SetPropertyBool("CLASSIFICATION::RightSidePercentage" NSSUBPROP_ISCHANGEABLE, tTrue);

	m_inputImageSet = tFalse;

}

cClassificationVisualizer::~cClassificationVisualizer()
{
}

tResult cClassificationVisualizer::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cClassificationVisualizer::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cClassificationVisualizer::PropertyChanged(const tChar* strName)
{

	RETURN_IF_FAILED(cFilter::PropertyChanged(strName)); 
 if (cString::IsEqual(strName, "DEBUG::DebutOutput")) 
 { 
 m_filterProperties.debugOutput = GetPropertyFloat("DEBUG::DebutOutput");  
 } 

	//associate the properties to the member
	if (cString::IsEqual(strName, "FILTERING::MinProbability"))
		m_filterProperties.minProbability = GetPropertyFloat("FILTERING::MinProbability");
	else if (cString::IsEqual(strName, "CLASSIFICATION::LeftSidePercentage"))
		m_filterProperties.LeftSidePercentage = GetPropertyFloat("CLASSIFICATION::LeftSidePercentage");
	else if (cString::IsEqual(strName, "CLASSIFICATION::RightSidePercentage"))
		m_filterProperties.RightSidePercentage = GetPropertyFloat("CLASSIFICATION::RightSidePercentage");

	RETURN_NOERROR;
}





tResult cClassificationVisualizer::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		//create pin for input
		RETURN_IF_FAILED(m_oClassificationPin.Create("classification", new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oClassificationPin));

		// GCL Output
		RETURN_IF_FAILED(m_oGCLOutputPin.Create("GCL", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oGCLOutputPin)); 

    }
    else if (eStage == StageNormal)
    {
    }

    else if (eStage == StageGraphReady)
    {
		
    }

    RETURN_NOERROR;
}


tResult cClassificationVisualizer::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	RETURN_IF_POINTER_NULL(pSource);
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		RETURN_IF_POINTER_NULL(pMediaSample);
		if (pSource == &m_oClassificationPin)
		{
			std::vector<cnnClassificationResult> classificationResults;
			classificationResults.resize(pMediaSample->GetSize() / sizeof(classificationResults));
			
			//get the date from the media sample
			tVoid* pIncomingData;
			if (IS_OK(pMediaSample->Lock((const tVoid**)&pIncomingData)))
			{
				//make copy
				memcpy(classificationResults.data(), pIncomingData, pMediaSample->GetSize());
				pMediaSample->Unlock(pIncomingData);
			}


			if (!classificationResults.empty())
			{
				// Transmit the detected objects to the output window
				transmitGCL(classificationResults);
			}
		}

	}


	RETURN_NOERROR;

}


tResult cClassificationVisualizer::ProcessVideo(IMediaSample* pSample)
{

	RETURN_IF_POINTER_NULL(pSample);
    // new image for result
	const tVoid* l_pSrcBuffer;
    
	//receiving data from input sample, and saving to TheInputImage
	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{
		//convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
		{
			//copy the data to matrix (make a copy, not change the sample content itself!)
			memcpy(m_inputImage.data, l_pSrcBuffer, m_sInputFormat.nSize);
			//or just set the data pointer of matrix because we create a new matrix later on
			// m_inputImage.data = (uchar*)(l_pSrcBuffer);
		}
		pSample->Unlock(l_pSrcBuffer);
	}
	
	m_inputImageSet = tTrue;
	
	RETURN_NOERROR;
}




// Transmit the found values through the gcl output pin
tResult cClassificationVisualizer::transmitGCL(std::vector<cnnClassificationResult> resultVector)
{
	
	
	IDynamicMemoryBlock* pGCLFrameAndText;
	cGCLWriter::GetDynamicMemoryBlock(pGCLFrameAndText);

	//set color
	cGCLWriter::StoreCommand(pGCLFrameAndText, GCL_CMD_FGCOL, cColor::Red.GetRGBA());

	//  TODO Check for camera resolution and set accordingly
	//cGCLWriter::StoreCommand(pGCLFrameAndText, GCL_CMD_DRAWLINE, 0, 260, 640, 260);
	
	for (std::vector<cnnClassificationResult>::iterator it = resultVector.begin(); it != resultVector.end(); it++)
	{
		if(it->probability * 100 > m_filterProperties.minProbability ){
			// cString name =  it->cnnClassificationDesc;
			// name += " (";
			// cString orientation =  it->orientation;
			// orientation += ")";
			cString description;
			if(str_comp(it -> cnnClassificationDesc, "person")) {
				description =  "person (" + (cString) it->orientation + ")";
			} else {
				description=  it->cnnClassificationDesc;
			}

			
			tInt width =  1920;
			tInt height =  1080;
			
			
			
			cGCLWriter::StoreCommand(pGCLFrameAndText, GCL_CMD_TEXT, (int) (it->xmin*width), (int) (it->ymin*height) -10, description.GetLength());
			cGCLWriter::StoreData(pGCLFrameAndText, description.GetLength(), description.GetPtr());				
			cGCLWriter::StoreCommand(pGCLFrameAndText, GCL_CMD_DRAWRECT, (int) (it->xmin*width), (int) (it->ymin*height) , (int) (it->xmax*width) , (int) (it->ymax*height));
			

			// Draw lines to show where the left, right and middle part of a crossing are.
			cGCLWriter::StoreCommand(pGCLFrameAndText, GCL_CMD_DRAWLINE, (int)(width* m_filterProperties.LeftSidePercentage), 0 ,(int)(width* m_filterProperties.LeftSidePercentage), height);	
			cGCLWriter::StoreCommand(pGCLFrameAndText, GCL_CMD_DRAWLINE, (int)(width* m_filterProperties.RightSidePercentage), 0 ,(int)(width* m_filterProperties.RightSidePercentage), height);

			//cout << (string) it->cnnClassificationDesc << endl;				
			string searchLabel ("person");			
			string currentLabel (it->cnnClassificationDesc);	
			// if(searchLabel.compare(currentLabel)== 0)
			// {				
						
			// 	dolls.push_back((it->ymax - it->ymin)*480);
			// }
			
		}
					
	}

	cGCLWriter::StoreCommand(pGCLFrameAndText, GCL_CMD_END);
	
	//alloc media sample and transmit it over output pin
	cObjectPtr<IMediaSample> pSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));
	RETURN_IF_FAILED(pSample->Update(_clock->GetStreamTime(),
									 pGCLFrameAndText->GetPtr(), (tInt)pGCLFrameAndText->GetSize(), IMediaSample::MSF_None));
	RETURN_IF_FAILED(m_oGCLOutputPin.Transmit(pSample));
	
	
	cGCLWriter::FreeDynamicMemoryBlock(pGCLFrameAndText);
	RETURN_NOERROR;
}

tResult cClassificationVisualizer::ProcessInputDepth(IMediaSample* pSample)
{
	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	const tVoid* l_pSrcBuffer;

	IplImage* oImg = cvCreateImage(cvSize(m_sInputFormatDepthImage.nWidth, m_sInputFormatDepthImage.nHeight), IPL_DEPTH_16U, 3);
	RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
	oImg->imageData = (char*)l_pSrcBuffer;
	Mat image(cvarrToMat(oImg));
	cvReleaseImage(&oImg);
	pSample->Unlock(l_pSrcBuffer);

	Mat m_depthImage = Mat(m_sInputFormatDepthImage.nHeight,m_sInputFormatDepthImage.nWidth,CV_16UC1,(tVoid*)l_pSrcBuffer,m_sInputFormatDepthImage.nBytesPerLine);
	//////////////
	// Transform image
	m_matImageDepth= m_depthImage(cv::Range(120, 360), cv::Range(160, 480)).clone(); //Cut Image
	//cv::FileStorage file("/home/aadc/Desktop/picture.txt", cv::FileStorage::WRITE);
	const char* Filename = "/home/aadc/Desktop/picture.txt";
	writeMatToFile(m_matImageDepth,Filename);
	// writeMatToFile(m_depthImage(cv::Range(120, 360), cv::Range(160, 480)).clone(),Filename);

	// Write to file!
	//file << m_matImageDepth;
	//cvtColor(image, image_GREY, CV_RGB2GRAY);// Grey Image 
	//RGBreceived=false;	
	
	RETURN_NOERROR;            
}






tResult cClassificationVisualizer::writeMatToFile(cv::Mat& m, const char* filename)
{
	ofstream fout(filename);
	// for(int i=0; i<m.rows; i++)
	// {
	//     for(int j=0; j<m.cols; j++)
	//     {
	//         fout<<m.at<float>(i,j)<<"\t";
	//     }
	//     fout<<endl;
	// }
	
	fout << m;
	fout.close();
	

	
	
	// namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	// imshow( "Display window", m );  
	
	RETURN_NOERROR;
}
