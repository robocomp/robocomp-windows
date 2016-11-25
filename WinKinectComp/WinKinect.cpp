
#include <iostream>
#include <map>
#include <math.h>
#include "WinKinect.h"

#define INITGUID
#include <guiddef.h>

// Static initializers
//LPCWSTR WinKinectBasics::GrammarFileName = L"SpeechBasics-D2D.grxml";

// This is the class ID we expect for the Microsoft Speech recognizer.
// Other values indicate that we're using a version of sapi.h that is
// incompatible with this sample.
DEFINE_GUID(CLSID_ExpectedRecognizer, 0x495648e7, 0xf7ab, 0x4267, 0x8e, 0x0f, 0xca, 0xfb, 0x7a, 0x33, 0xc1, 0x60);

/// <summary>
/// Constructor
/// </summary>
WinKinectBasics::WinKinectBasics() :
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0),
    m_pKinectSensor(nullptr),
	m_pCoordinateMapper(nullptr),
    m_pBodyFrameReader(nullptr),
	m_pColorFrameReader(nullptr),
    m_pColorRGBX(nullptr),
	m_pDepthFrameReader(nullptr),
	m_pDepthRGBX(nullptr),
	m_pAudioBeam(nullptr),
    m_pAudioStream(nullptr),
    m_p16BitAudioStream(nullptr),
    m_hSensorNotification(reinterpret_cast<WAITABLE_HANDLE>(INVALID_HANDLE_VALUE)),
    m_pSpeechStream(nullptr),
    m_pSpeechRecognizer(nullptr),
    m_pSpeechContext(nullptr),
    m_pSpeechGrammar(nullptr),
    m_hSpeechEvent(INVALID_HANDLE_VALUE),
	cameraSpacePoints(nullptr)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

	for (int i = 0; i < BODY_COUNT; i++)
    {
        m_pFaceFrameSources[i] = nullptr;
        m_pFaceFrameReaders[i] = nullptr;
    }


	// create heap storage for color pixel data in RGBX format
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	 // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	cameraSpacePoints= new CameraSpacePoint[cColorWidth*cColorHeight];

	started = false;
	executing = false;
	n_iter = 0;
	idioma=2; //1:Inglés; 2:Español
	test=2; //1:Barthel; 2:Mimental;

	changeConfig=false;
	changeQuestion=false;


}
/// <summary>
/// Destructor
/// </summary>
WinKinectBasics::~WinKinectBasics()
{

    if (m_pColorRGBX)
    {
        delete [] m_pColorRGBX;
        m_pColorRGBX = nullptr;
    }

	 if (m_pDepthRGBX)
    {
        delete [] m_pDepthRGBX;
        m_pDepthRGBX = NULL;
    }

    // clean up Direct2D
    //SafeRelease(m_pD2DFactory);
	 // done with face sources and readers
    for (int i = 0; i < BODY_COUNT; i++)
    {
        SafeRelease(m_pFaceFrameSources[i]);
        SafeRelease(m_pFaceFrameReaders[i]);		
    }

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

	// done with color frame reader
    SafeRelease(m_pColorFrameReader);

	// done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

	 //16 bit Audio Stream
    if (NULL != m_p16BitAudioStream)
    {
        delete m_p16BitAudioStream;
        m_p16BitAudioStream = NULL;
    }
    SafeRelease(m_pAudioStream);
    SafeRelease(m_pAudioBeam);

    SafeRelease(m_pKinectSensor);

	//ofile.close();
	//... when done, free the BSTR
	
}



void WinKinectBasics::initializingData()
{

	ready = false;

	n_iter = -1; 
	m_nStartTime = 0;


	// Get and initialize the default Kinect sensor
    InitializeDefaultSensor();

}

/*HRESULT WinKinectBasics::startAudioThread()
{

	hThreadArray[0] = CreateThread( NULL,                   // default security attributes
									0,                      // use default stack size  
									audioCapture,       // thread function name
									this,          // argument to thread function 
									0,                      // use default creation flags 
									&dwThreadIdArray[0]);   // returns the thread identifier 

	if (hThreadArray[0] == NULL) 
	{
		ExitProcess(3);
	}
	
	return S_OK;
}*/
void WinKinectBasics::stoppingCapture()
{
	{
		std::unique_lock<std::mutex> lck(mtx);
		ready = true;
		cv.notify_all();
	}
	//waitForAudioThread();
	/*if (INVALID_HANDLE_VALUE != waveFile)
	{
		CloseHandle(waveFile);
	}*/

	//delete capturer;
	//SafeRelease(device);
	CoUninitialize();
	//SysFreeString(outputDirL);
	executing = false;
	//if(storin_ssd) 
		//moveDirectory();
}


/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void WinKinectBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies,RGBQUAD* pBuffer,int nWidth, int nHeight)
{

	HRESULT hr;
	
	RoboCompMSKBody::PersonList people;
	RoboCompMSKFace::DetailedFaceMap myfaces;

	int width = cColorWidth;
	int height = cColorHeight;
	//static INT64 time_old=0;
	INT64 tiempo=nTime/10000;

	//getCurrentTime();
		//setStoreDataDisk();
    for (int i = 0; i < nBodyCount; ++i)
     {

            IBody* pBody = ppBodies[i];
            if (pBody)
            {
                BOOLEAN bTracked = false;
                hr = pBody->get_IsTracked(&bTracked);

				RoboCompMSKBody::TPerson person;

                if (SUCCEEDED(hr) && bTracked)
                {

					//std::cout<<"-------------------------------------------------TRACKEANDO"<<std::endl;
					
					bool valid;

					RoboCompMSKFace::DetailedFace myface = ProcessFaces(ppBodies,i,valid);

					//time_old=nTime;
					tracking = true;
                    Joint joints[JointType_Count]; 
	                UINT64 trackingId;
					
					cout<<"Tiempo de captura de los joints: "<<tiempo<<endl;

                    hr = pBody->GetJoints(_countof(joints), joints);
                    if (SUCCEEDED(hr))
                    {
						
						RoboCompMSKBody::JointList jointList;
						for(int j=0; j< _countof(joints); j++){

							RoboCompMSKBody::Joint joint;
							int state=(int)joints[j].TrackingState;
							joint.state=(RoboCompMSKBody::JointTrackingState)state;
							joint.Position.X=joints[j].Position.X;
							joint.Position.Y=joints[j].Position.Y;
							joint.Position.Z=joints[j].Position.Z;
							
							person.joints.insert(std::pair<RoboCompMSKBody::JointType,RoboCompMSKBody::Joint>((RoboCompMSKBody::JointType)joints[j].JointType,joint));
						}
						person.Position.X=joints[ JointType_SpineMid].Position.X;
						person.Position.Y=joints[ JointType_SpineMid].Position.Y;
						person.Position.Z=joints[ JointType_SpineMid].Position.Z;
						
						//person.TrackingId=i;
						cout<<"Posicion del dedo indice de la mano derecha:  "<<joints[JointType_HandTipRight].Position.X<<"  "<<joints[JointType_HandTipRight].Position.Y<<"  "<<joints[JointType_HandTipRight].Position.Z<<endl;

                    }
					person.trackedState=bTracked;
					person.TrackingId=i;
					people.insert(std::pair<int,RoboCompMSKBody::TPerson>(i,person));

					cout<<"Trackeando cuerpo:   "<<person.TrackingId<<endl;

					if(people.count(i)>0)
							cout << people[i].TrackingId << endl; 

					//bool valid;

					//RoboCompMSKFace::DetailedFace myface = ProcessFaces(ppBodies,i,valid);
						
					if (valid){

						myfaces.insert(std::pair<int,RoboCompMSKFace::DetailedFace>(i,myface));
						if(myfaces.count(i)>0)
							cout << myfaces[i].identifier << endl; 
					}
						

                }
            }
     

        // Device lost, need to recreate the render target
        // We'll dispose it now and retry drawing
        if (D2DERR_RECREATE_TARGET == hr)
        {
            hr = S_OK;
        }
    }

	if(people.size()>0){
		//cout << "Intentando publicar esqueleto..." << endl;
		try
		{
			bodyPrx->newMSKBodyEvent(people,tiempo);
			for(auto i=people.begin(); i!=people.end();i++)
				cout << "Tracking id: " << i->second.TrackingId << endl; 
			//cout << "Se ha publicado bien el esqueleto" << endl;
		}catch(Ice::Exception e)
		{
			cout << "ERRORRR" << endl;
		}
	}

	if(myfaces.size()>0){
		//cout << "Intentando publicar caras..." << endl;
		try
		{
			facePrx->newFaceAvailable(myfaces,0);
			//cout << "Se han publicado bien las caras" << endl;
		}catch(Ice::Exception e)
		{
			//cout << "ERRORRR publicando caras" << endl;
		}

	}

}



/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F WinKinectBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
    // Calculate the body's position on the screen
    //DepthSpacePoint depthPoint = {0};
	ColorSpacePoint colorPoint;
    m_pCoordinateMapper->MapCameraPointToColorSpace(bodyPoint, &colorPoint);

    float screenPointX = static_cast<float> (colorPoint.X);//(depthPoint.X * width) / cDepthWidth;
	float screenPointY = static_cast<float> (colorPoint.Y);//(depthPoint.Y * height) / cDepthHeight;

    return D2D1::Point2F(screenPointX, screenPointY);
}




/// <summary>
/// Handle new color data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// </summary>
void WinKinectBasics::ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight) 
{
   
	 
	 if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight)){


		 //cout<<"dentro"<<endl;
		
		 BYTE *mybuffer=reinterpret_cast<BYTE*>(pBuffer);
		 std::vector<BYTE> miimagen(mybuffer,mybuffer+nWidth*nHeight*sizeof(RGBQUAD));
		 imagen=miimagen;

		 imageStruct.image=imagen;
		 imageStruct.height=nHeight;
		 imageStruct.width=nWidth;


	 }
		
}

void    WinKinectBasics::ProcessDepth(INT64 nDepthTime, UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, USHORT nMinDepth, USHORT nMaxDepth){

	if(pDepthBuffer && (nDepthWidth==cDepthWidth) && (nDepthHeight==cDepthHeight)){

		//mypDepthBuffer=pDepthBuffer;
		//cameraSpacePoints= new CameraSpacePoint[cColorWidth*cColorHeight];
				
		HRESULT hr=m_pCoordinateMapper-> MapColorFrameToCameraSpace(nDepthWidth*nDepthHeight,(UINT16*)pDepthBuffer,cColorWidth*cColorHeight,cameraSpacePoints);

		short *mybuffer=(short*)pDepthBuffer;
		std::vector<short> miimagen(pDepthBuffer,pDepthBuffer+(nDepthWidth*nDepthHeight));

		depthStruct.image=miimagen;
		depthStruct.height=nDepthHeight;
		depthStruct.width=nDepthWidth;
	}
}



/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT WinKinectBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    IAudioSource* pAudioSource = NULL;
    IAudioBeamList* pAudioBeamList = NULL;
    BOOLEAN sensorState = TRUE;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);

    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {

		hr = m_pKinectSensor->SubscribeIsAvailableChanged(&m_hSensorNotification);

        // Initialize Kinect and get color, body and face readers
        IColorFrameSource* pColorFrameSource = nullptr;
        IBodyFrameSource* pBodyFrameSource = nullptr;
		// Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = nullptr;

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->Open();
		}
     
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }

		 if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        if (SUCCEEDED(hr))
        {
            // create a face frame source + reader to track each body in the fov
            for (int i = 0; i < BODY_COUNT; i++)
            {
                if (SUCCEEDED(hr))
                {
                    // create the face frame source by specifying the required face frame features
                    hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
                }
                if (SUCCEEDED(hr))
                {
                    // open the corresponding reader
                    hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
                }				
            }
        }

		 if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

	/*	if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_AudioSource(&pAudioSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pAudioSource->get_AudioBeams(&pAudioBeamList);
		}

		if (SUCCEEDED(hr))
		{        
			hr = pAudioBeamList->OpenAudioBeam(0, &m_pAudioBeam);
		}

		if (SUCCEEDED(hr))
		{        
			hr = m_pAudioBeam->OpenInputStream(&m_pAudioStream);
			m_p16BitAudioStream = new KinectAudioStream(m_pAudioStream);
		}

		if (FAILED(hr))
		{
			cout<<"Fallando al inicializar el audio"<<endl;
		}

		SafeRelease(pAudioBeamList);
		SafeRelease(pAudioSource);
		*/
        SafeRelease(pDepthFrameSource);
        SafeRelease(pColorFrameSource);
        SafeRelease(pBodyFrameSource);
    }
 

    return hr;
}






/// <summary>
/// Processes new face frames
/// </summary>
RoboCompMSKFace::DetailedFace WinKinectBasics::ProcessFaces(IBody** ppBodies, int iFace, bool &valid)
{
    HRESULT hr;
    
	RoboCompMSKFace::DetailedFace myface;
    
    // retrieve the latest face frame from this reader
    IFaceFrame* pFaceFrame = nullptr;
    hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

    BOOLEAN bFaceTracked = false;
    if (SUCCEEDED(hr) && nullptr != pFaceFrame)
    {
        // check if a valid face is tracked in this face frame
        hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
    }

	valid=false;

    if (SUCCEEDED(hr))
    {
        if (bFaceTracked)
        {

			UINT64 trackingId;
            IFaceFrameResult* pFaceFrameResult = nullptr;
            RectI faceBox = {0};
       
            Vector4 faceRotation;
            DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
            PointF facePoints[FacePointType::FacePointType_Count];

            hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

            // need to verify if pFaceFrameResult contains data before trying to access it
            if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
            {

				
                hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);

				if(SUCCEEDED(hr)){
					hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
				}

				if (SUCCEEDED(hr)){
					  hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
				}
				  
				if (SUCCEEDED(hr))
                {
                    hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);
				}
				if (SUCCEEDED(hr)){

					if (ValidateFaceBoxAndPoints(&faceBox, facePoints))
					{

							valid=true;

							myface.top=faceBox.Top;
							myface.bottom=faceBox.Bottom;
							myface.right=faceBox.Right;
							myface.left=faceBox.Left;

							PointF nose=facePoints[FacePointType_Nose];
							
							//int leftEar_j=myface.right;
							//int leftEar_i=(myface.top+myface.bottom)/2;
							//int rightEar_i=(myface.top+myface.bottom)/2;
							//int rightEar_j=myface.left;

							 CameraSpacePoint nose3D= cameraSpacePoints[(int)nose.Y*cColorWidth+(int)nose.X];
							 myface.nose.x=nose3D.X;
							 myface.nose.y=nose3D.Y;
							 myface.nose.z=nose3D.Z;

							// CameraSpacePoint leftEar3D= cameraSpacePoints[leftEar_i*cColorWidth+leftEar_j];
							 myface.leftEar.x=nose3D.X-0.1;
							 myface.leftEar.y=nose3D.Y;
							 myface.leftEar.z=nose3D.Z+0.05;

							// CameraSpacePoint rightEar3D= cameraSpacePoints[rightEar_i*cColorWidth+rightEar_j];
							 myface.rightEar.x=nose3D.X+0.1;
							 myface.rightEar.y=nose3D.Y;
							 myface.rightEar.z=nose3D.Z+0.05;

							 cout<<"Coordenadas de la nariz: "<<myface.nose.x<<"   "<<myface.nose.y<<"   "<<myface.nose.z<<endl;
							 cout<<"Coordenadas de la oreja derecha: "<<myface.rightEar.x<<"   "<<myface.rightEar.y<<"   "<<myface.rightEar.z<<endl;
							 cout<<"Coordenadas de la oreja izquierda: "<<myface.leftEar.x<<"   "<<myface.leftEar.y<<"   "<<myface.leftEar.z<<endl;
							 
							 int pitch, yaw, roll;
							 ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);

							 myface.pitch=pitch;
							 myface.yaw=yaw;
							 myface.roll=roll;

							 for (int iProperty = 0; iProperty < FaceProperty::FaceProperty_Count; iProperty++)
							{
						
								if(iProperty==FaceProperty::FaceProperty_LeftEyeClosed){
										if(faceProperties[iProperty]==DetectionResult::DetectionResult_Yes){
											myface.leftEyeClosed=true;
										}else {
											myface.leftEyeClosed=false;
										}
								}

								if(iProperty==FaceProperty::FaceProperty_RightEyeClosed){
										if(faceProperties[iProperty]==DetectionResult::DetectionResult_Yes){
											myface.rightEyeClosed=true;
										}else {
											myface.rightEyeClosed=false;
										}
								}

							}

							int alto =myface.bottom-myface.top+1;
							int ancho=myface.right-myface.left+1;
							//std::vector<byte> myfaceimage;//(alto*ancho*3,0);
					
							//int cont=0;
					

							for(int j=myface.top; j<=myface.bottom; j++){
								for(int k=myface.left; k<=myface.right; k++){

									if(j>=0 && j<cColorHeight && k>=0 && k<cColorWidth){

										myface.faceImage.image.push_back(imagen[j*ancho+k]);
										myface.faceImage.image.push_back(imagen[j*ancho+k+1]);
										myface.faceImage.image.push_back(imagen[j*ancho+k+2]);

										//cont=cont+3;
									}
								}
							}
						
						

							myface.faceImage.width=ancho;
							myface.faceImage.height=alto;
						
							cout<<"TRACKEANDO CARA:   "<<iFace<<endl;	

							myface.identifier=iFace;
					}

				}
			/*	myface.top=faceBox.Top;
				myface.bottom=faceBox.Bottom;
				myface.right=faceBox.Right;
				myface.left=faceBox.Left;

				if (SUCCEEDED(hr))
				{
					 hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
					 bool seguir=false;

					 if (facePoints[FacePointType_Nose].X!=0 || facePoints[FacePointType_Nose].Y!=0)
						 seguir=true;

					 if(seguir){
						 PointF nose=facePoints[FacePointType_Nose];
						 int leftEar_j=myface.right;
						 int leftEar_i=(myface.top+myface.bottom)/2;
						 int rightEar_i=(myface.top+myface.bottom)/2;
						 int rightEar_j=myface.left;

					 
					
						 if (SUCCEEDED(hr))
						 {
						
								 if (cameraSpacePoints!=NULL)
								 {
									 CameraSpacePoint nose3D= cameraSpacePoints[(int)nose.Y*cColorWidth+(int)nose.X];
									 myface.nose.x=nose3D.X;
									 myface.nose.y=nose3D.Y;
									 myface.nose.z=nose3D.Z;

									 CameraSpacePoint leftEar3D= cameraSpacePoints[leftEar_i*cColorWidth+leftEar_j];
									 myface.leftEar.x=leftEar3D.X;
									 myface.leftEar.y=leftEar3D.Y;
									 myface.leftEar.z=leftEar3D.Z;

									 CameraSpacePoint rightEar3D= cameraSpacePoints[rightEar_i*cColorWidth+rightEar_j];
									 myface.rightEar.x=rightEar3D.X;
									 myface.rightEar.y=rightEar3D.Y;
									 myface.rightEar.z=rightEar3D.Z;


									 cout<<"Coordenadas de la nariz: "<<myface.nose.x<<"--"<<myface.nose.y<<"--"<<myface.nose.z<<endl;
								 }else{
									 myface.nose.x=0;
									 myface.nose.y=0;
									 myface.nose.z=0;

									 myface.leftEar.x=0;
									 myface.leftEar.y=0;
									 myface.leftEar.z=0;

									 myface.rightEar.x=0;
									 myface.rightEar.y=0;
									 myface.rightEar.z=0;

								 }
						 
						 }
					 }else{

						myface.nose.x=0;
						myface.nose.y=0;
						myface.nose.z=0;

						myface.leftEar.x=0;
						myface.leftEar.y=0;
						myface.leftEar.z=0;

						myface.rightEar.x=0;
						myface.rightEar.y=0;
						myface.rightEar.z=0;
					 }
				}

                if (SUCCEEDED(hr))
                {
                    hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
					 // extract face rotation in degrees as Euler angles
					int pitch, yaw, roll;
					ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);

					myface.pitch=pitch;
					myface.yaw=yaw;
					myface.roll=roll;

                }

                if (SUCCEEDED(hr))
                {
                    hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);

					for (int iProperty = 0; iProperty < FaceProperty::FaceProperty_Count; iProperty++)
					{
						
						if(iProperty==FaceProperty::FaceProperty_LeftEyeClosed){
							if(faceProperties[iProperty]==DetectionResult::DetectionResult_Yes){
								myface.leftEyeClosed=true;
							}else {
								myface.leftEyeClosed=false;
							}
						}

						if(iProperty==FaceProperty::FaceProperty_RightEyeClosed){
							if(faceProperties[iProperty]==DetectionResult::DetectionResult_Yes){
								myface.rightEyeClosed=true;
							}else {
								myface.rightEyeClosed=false;
							}
						}

					}

					int alto =myface.bottom-myface.top+1;
					int ancho=myface.right-myface.left+1;
					//std::vector<byte> myfaceimage;//(alto*ancho*3,0);
					
					//int cont=0;
					

					for(int j=myface.top; j<=myface.bottom; j++){
						for(int k=myface.left; k<=myface.right; k++){

							if(j>=0 && j<cColorHeight && k>=0 && k<cColorWidth){

								

								myface.faceImage.image.push_back(imagen[j*ancho+k]);
								myface.faceImage.image.push_back(imagen[j*ancho+k+1]);
								myface.faceImage.image.push_back(imagen[j*ancho+k+2]);

								//cont=cont+3;
							}
						}
					}
						
						

					myface.faceImage.width=ancho;
					myface.faceImage.height=alto;
					//myface.faceImage.image=myfaceimage;

					valid=true;
				}

				
                
				
               	cout<<"TRACKEANDO CARA:   "<<iFace<<endl;	

				myface.identifier=iFace;*/
			}

            SafeRelease(pFaceFrameResult);	
			
        }
        else 
        {	
              
                // check if the corresponding body is tracked 
                // if this is true then update the face frame source to track this body
                IBody* pBody = ppBodies[iFace];
                if (pBody != nullptr)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    UINT64 bodyTId;
                    if (SUCCEEDED(hr) && bTracked)
                    {
                        // get the tracking ID of this body
                        hr = pBody->get_TrackingId(&bodyTId);
                        if (SUCCEEDED(hr))
                        {
                            // update the face frame source with the tracking ID
                            m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
                        }
                    }
                }
            
        }
        			

    SafeRelease(pFaceFrame);
 }

  return myface;
}

/// <summary>
/// Converts rotation quaternion to Euler angles 
/// And then maps them to a specified range of values to control the refresh rate
/// </summary>
/// <param name="pQuaternion">face rotation quaternion</param>
/// <param name="pPitch">rotation about the X-axis</param>
/// <param name="pYaw">rotation about the Y-axis</param>
/// <param name="pRoll">rotation about the Z-axis</param>
void WinKinectBasics::ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll)
{
    double x = pQuaternion->x;
    double y = pQuaternion->y;
    double z = pQuaternion->z;
    double w = pQuaternion->w;

    // convert face rotation quaternion to Euler angles in degrees		
    double dPitch, dYaw, dRoll;
    dPitch = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0;
    dYaw = asin(2 * (w * y - x * z)) / M_PI * 180.0;
    dRoll = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0;

    // clamp rotation values in degrees to a specified range of values to control the refresh rate
    double increment = c_FaceRotationIncrementInDegrees; 
    *pPitch = static_cast<int>(floor((dPitch + increment/2.0 * (dPitch > 0 ? 1.0 : -1.0)) / increment) * increment);
    *pYaw = static_cast<int>(floor((dYaw + increment/2.0 * (dYaw > 0 ? 1.0 : -1.0)) / increment) * increment);
    *pRoll = static_cast<int>(floor((dRoll + increment/2.0 * (dRoll > 0 ? 1.0 : -1.0)) / increment) * increment);
}


DWORD WINAPI audioCapture( LPVOID lpParam ) {

	HANDLE hStdout;

	const int maxEventCount = 2;
    int eventCount = 1;
    HANDLE hEvents[maxEventCount];

	int iteracion=0;

    
	// Cast the parameter to the correct data type.
    // The pointer is known to be valid because 
    // it was checked for NULL before the thread was created.
 
    WinKinectBasics* wkb = (WinKinectBasics*)lpParam;

	if (CLSID_ExpectedRecognizer != CLSID_SpInprocRecognizer)
    {
       cout<<"This sample was compiled against an incompatible version of sapi.h. Please ensure that Microsoft Speech SDK and other sample requirements are installed and then rebuild application."<<endl;

        return EXIT_FAILURE;
    }

	HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

    if (SUCCEEDED(hr))
    {
		 {

			while(!wkb->salir){
				
				if(wkb->changeConfig || wkb->changeQuestion){
					
					wkb->mtx_I.lock();
					cout<<"Empieza a cargar"<<endl;
					wkb->changeConfig=false;
					wkb->changeQuestion=false;
					hr = wkb->InitializeSpeech();

					wkb->mtx_I.unlock();	

					if (SUCCEEDED(hr))
					{
							cout<<"Speech inicializado"<<endl;
							
					}
					else
					{
							cout<<"Speech Initialization Failed"<<endl;
					}
						
				}
				
				if (wkb->m_hSpeechEvent != INVALID_HANDLE_VALUE)
				{
					hEvents[1] = wkb->m_hSpeechEvent;
					eventCount = 2;
				}

				hEvents[0] = reinterpret_cast<HANDLE>(wkb->m_hSensorNotification);
				 // Check to see if we have either a message (by passing in QS_ALLINPUT)
				// Or sensor notification (hEvents[0])
				// Or a speech event (hEvents[1])
				DWORD waitResult = MsgWaitForMultipleObjectsEx(eventCount, hEvents, 500, QS_ALLINPUT, MWMO_INPUTAVAILABLE);
				//DWORD waitResult = MsgWaitForMultipleObjectsEx(eventCount, hEvents, INFINITE, QS_ALLINPUT, MWMO_INPUTAVAILABLE);

				//cout<<waitResult<<endl;

				switch (waitResult)
				{
					case WAIT_OBJECT_0:
					{
						BOOLEAN sensorState = FALSE;

						// Getting the event data will reset the event.
						IIsAvailableChangedEventArgs* pEventData = nullptr;
						if (FAILED(wkb->m_pKinectSensor->GetIsAvailableChangedEventData(wkb->m_hSensorNotification, &pEventData)))
						{
							cout<<"Failed to get sensor availability."<<endl;
							break;
						}

						pEventData->get_IsAvailable(&sensorState);
						SafeRelease(pEventData);

						if (sensorState == FALSE)
						{
							//cout<<"Sensor has been disconnected - attach Sensor"<<endl;
						}
						else
						{
							HRESULT hr = S_OK;

							if (wkb->m_pSpeechRecognizer == NULL)
							{
								wkb->mtx_I.lock();
								hr = wkb->InitializeSpeech();	
								wkb->mtx_I.unlock();
							
								if (SUCCEEDED(hr))
								{
									cout<<"Speech inicializado"<<endl;
								
								}
								else
								{
									cout<<"Speech Initialization Failed"<<endl;
								}
							}
						}
					}
					break;
				
				case WAIT_OBJECT_0 +1 :
						//if (eventCount == 2)
				{
					if(wkb->ProcessSpeech() && (wkb->changeConfig || wkb->changeQuestion))
					{
						wkb->mtx_I.lock();

						wkb->changeConfig=false;
						wkb->changeQuestion=false;
						wkb->InitializeSpeech();

						wkb->mtx_I.unlock();
					}						
				}
				break;
			}
			}
        }

		if (NULL != wkb->m_p16BitAudioStream)
        {
            wkb->m_p16BitAudioStream->SetSpeechState(false);
        }

        if (NULL != wkb->m_pSpeechRecognizer)
        {
            wkb->m_pSpeechRecognizer->SetRecoState(SPRST_INACTIVE_WITH_PURGE);

            //cleanup here
            SafeRelease(wkb->m_pSpeechStream);
            SafeRelease(wkb->m_pSpeechRecognizer);
            SafeRelease(wkb->m_pSpeechContext);
            SafeRelease(wkb->m_pSpeechGrammar);
        }

        CoUninitialize();
    }
 
	return 0;
}

HRESULT WinKinectBasics::startAudioThread()
{

	hThreadArray[0] = CreateThread( NULL,                   // default security attributes
									0,                      // use default stack size  
									audioCapture,       // thread function name
									this,          // argument to thread function 
									0,                      // use default creation flags 
									&dwThreadIdArray[0]);   // returns the thread identifier 

	if (hThreadArray[0] == NULL) 
	{
		ExitProcess(3);
	}
	
	return S_OK;
}

HRESULT WinKinectBasics::waitForAudioThread()
{
	 WaitForMultipleObjects(MAX_THREADS, hThreadArray, TRUE, INFINITE);
	 return S_OK;
}


HRESULT WinKinectBasics::InitializeSpeech(){

	// Audio Format for Speech Processing
    WORD AudioFormat = WAVE_FORMAT_PCM;
    WORD AudioChannels = 1;
    DWORD AudioSamplesPerSecond = 16000;
    DWORD AudioAverageBytesPerSecond = 32000;
    WORD AudioBlockAlign = 2;
    WORD AudioBitsPerSample = 16;

    WAVEFORMATEX wfxOut = {AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0};


	//if (NULL != m_p16BitAudioStream)
    //{
      //  m_p16BitAudioStream->SetSpeechState(false);
    //}

    if (NULL != m_pSpeechRecognizer)
    {
        m_pSpeechRecognizer->SetRecoState(SPRST_INACTIVE_WITH_PURGE);

        //cleanup here
        SafeRelease(m_pSpeechStream);
        SafeRelease(m_pSpeechRecognizer);
        SafeRelease(m_pSpeechContext);
        SafeRelease(m_pSpeechGrammar);
    }

  
       HRESULT hr = CreateSpeechRecognizer();
    

    if (FAILED(hr))
    {
        cout<<"Could not create speech recognizer. Please ensure that Microsoft Speech SDK and other sample requirements are installed."<<endl;
        return hr;
    }

    hr = LoadSpeechGrammar();

    if (FAILED(hr))
    {
        cout<<"Could not load speech grammar. Please ensure that grammar configuration file was properly deployed."<<endl;
        return hr;
    }

	
    hr = StartSpeechRecognition();




    if (FAILED(hr))
    {
        cout<<"Could not start recognizing speech."<<endl;
        return hr;
    }

    return hr;
}

/// <summary>
/// Create speech recognizer that will read Kinect audio stream data.
/// </summary>
/// <returns>
/// <para>S_OK on success, otherwise failure code.</para>
/// </returns>
HRESULT WinKinectBasics::CreateSpeechRecognizer()
{
    ISpObjectToken *pEngineToken = NULL;

    HRESULT hr = CoCreateInstance(CLSID_SpInprocRecognizer, NULL, CLSCTX_ALL, __uuidof(ISpRecognizer), (void**)&m_pSpeechRecognizer);

    if (SUCCEEDED(hr))
    {
       ISpObjectToken* audioToken; 
       ISpObjectTokenCategory* audioTokenCategory; 
            
	   hr = CoCreateInstance(CLSID_SpObjectTokenCategory, NULL, CLSCTX_ALL, IID_ISpObjectTokenCategory,  (void**)&audioTokenCategory);            
	   
	   if (SUCCEEDED(hr)) { 
                hr = audioTokenCategory->SetId(SPCAT_AUDIOIN, TRUE); 
       }            
	  if (SUCCEEDED(hr)) { 
                WCHAR * tokenID; 
                 hr = audioTokenCategory->GetDefaultTokenId(&tokenID); 
                 if (SUCCEEDED(hr)) { 
                     hr = CoCreateInstance(CLSID_SpObjectToken, NULL, CLSCTX_ALL, IID_ISpObjectToken, (void**)&audioToken); 
                     if (SUCCEEDED(hr)) { 
                         hr = audioToken->SetId(NULL, tokenID, FALSE); 
                    } 
                     ::CoTaskMemFree(tokenID); 
                 } 
        } 
        if (SUCCEEDED(hr)) { 
               hr = static_cast<ISpRecognizer*>(m_pSpeechRecognizer)->SetInput(audioToken, TRUE); 
        } 
 

		
		//  m_pSpeechRecognizer->SetInput(m_pSpeechStream, TRUE);

        // If this fails here, you have not installed the acoustic models for Kinect

		if(config.language=="english"){

			hr = SpFindBestToken(SPCAT_RECOGNIZERS, L"Language=409;Kinect=True", NULL, &pEngineToken); //Ingles

		}else if(config.language=="spanish"){

			hr = SpFindBestToken(SPCAT_RECOGNIZERS, L"Language=40a;Kinect=True", NULL, &pEngineToken); //Español
		}

        if (SUCCEEDED(hr))
        {
            m_pSpeechRecognizer->SetRecognizer(pEngineToken);
            hr = m_pSpeechRecognizer->CreateRecoContext(&m_pSpeechContext);

            // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model. 
            // This will prevent recognition accuracy from degrading over time.
            if (SUCCEEDED(hr))
            {
                hr = m_pSpeechRecognizer->SetPropertyNum(L"AdaptationOn", 0);                
            }
        }
    }
    SafeRelease(pEngineToken);
    return hr;
}

/// <summary>
/// Load speech recognition grammar into recognizer.
/// </summary>
/// <returns>
/// <para>S_OK on success, otherwise failure code.</para>
/// </returns>
HRESULT WinKinectBasics::LoadSpeechGrammar()
{
    HRESULT hr = m_pSpeechContext->CreateGrammar(1, &m_pSpeechGrammar);

    if (SUCCEEDED(hr))
    {
		
		wstring gfile=L"No grammar";
	
		if(config.testType=="barthel" && config.language=="english"){

			GrammarFileName=L"Barthel-en.grxml";
		}
		else if(config.testType=="barthel" && config.language=="spanish"){

			GrammarFileName=L"Barthel-es.grxml";
		}else if(config.testType=="minimental" && config.language=="spanish"){
			
			gfile= L"MMSE-es_" + to_wstring(questionNumber) + L".grxml";

			GrammarFileName= gfile.c_str();

		}else{
			cout<<"LoadSpeechGrammar: Error en config, ninguna gramática seleccionada"<<endl;
		}
		

		wcout<<"Gramatica: "<<gfile.c_str()<<endl;
        // Populate recognition grammar from file
        hr = m_pSpeechGrammar->LoadCmdFromFile(GrammarFileName, SPLO_STATIC);
    }

    return hr;
}

/// <summary>
/// Start recognizing speech asynchronously.
/// </summary>
/// <returns>
/// <para>S_OK on success, otherwise failure code.</para>
/// </returns>
HRESULT WinKinectBasics::StartSpeechRecognition()
{
    HRESULT hr = S_OK;

    // Specify that all top level rules in grammar are now active
    hr = m_pSpeechGrammar->SetRuleState(NULL, NULL, SPRS_ACTIVE);
    if (FAILED(hr))
    {
        return hr;
    }

    // Specify that engine should always be reading audio
    hr = m_pSpeechRecognizer->SetRecoState(SPRST_ACTIVE_ALWAYS);
    if (FAILED(hr))
    {
        return hr;
    }

    // Specify that we're only interested in receiving recognition events
    hr = m_pSpeechContext->SetInterest(SPFEI(SPEI_RECOGNITION), SPFEI(SPEI_RECOGNITION));
    if (FAILED(hr))
    {
        return hr;
    }

    // Ensure that engine is recognizing speech and not in paused state
    hr = m_pSpeechContext->Resume(0);
    if (FAILED(hr))
    {
        return hr;
    }

	hr=m_pSpeechContext->SetNotifyWin32Event();


    m_hSpeechEvent = m_pSpeechContext->GetNotifyEventHandle();

    return hr;
}

/// <summary>
/// Process recently triggered speech recognition events.
/// </summary>
bool WinKinectBasics::ProcessSpeech()
{
    bool ps_result = false;
	float ConfidenceThreshold = 0.3f;

    SPEVENT curEvent = {SPEI_UNDEFINED, SPET_LPARAM_IS_UNDEFINED, 0, 0, 0, 0};
    ULONG fetched = 0;
    HRESULT hr = S_OK;

	RoboCompMSKASR::TSentence sentence;

    m_pSpeechContext->GetEvents(1, &curEvent, &fetched);
    while (fetched > 0)
    {
        switch (curEvent.eEventId)
        {
	//	case SPEI_FALSE_RECOGNITION:
	//		cout<<" FALSE RECOGNITION!"<<endl;
	//		break;
        case SPEI_RECOGNITION:
	//		cout<<"SPEI RECOGNITION!!!!!!!"<<endl;
            if (SPET_LPARAM_IS_OBJECT == curEvent.elParamType)
            {
                // this is an ISpRecoResult
                ISpRecoResult* result = reinterpret_cast<ISpRecoResult*>(curEvent.lParam);
                SPPHRASE* pPhrase = NULL;

                hr = result->GetPhrase(&pPhrase);
                if (SUCCEEDED(hr))
                {
			//		cout<<"Ha habido un get frase"<<endl;
                    if ((pPhrase->pProperties != NULL) && (pPhrase->pProperties->pFirstChild != NULL))
                    {
                        const SPPHRASEPROPERTY* pSemanticTag = pPhrase->pProperties->pFirstChild;
						cout<<"Confidence value"<<pSemanticTag->SREngineConfidence;

						// para el 'ni sí, ni no, ni pero' del minimental se manda la respuesta siempre, buena o no. El umbral de confianza se pone más restrictivo
						if ( (config.testType=="minimental")&&(questionNumber==17) ) 
						{
							ConfidenceThreshold = 0.6f;
						}

						if (pSemanticTag->SREngineConfidence > ConfidenceThreshold)
						{
							//cout<<pSemanticTag->pszValue<<endl;
							//wprintf (L"%ls\n",pSemanticTag->pszValue);

							std::vector<string> frase_aux;
							string palabra;

							std::wstring ws = pSemanticTag->pszValue;
							int wlen = (int)ws.length();
							// get needed space
							int slen  = (int)wcstombs(NULL, &ws[0], 2*wlen);
           
							if (slen > 0) 
							{
								palabra.resize(slen, '\0'); 
								wcstombs(&palabra[0], &ws[0], slen);
							}

							frase_aux.push_back(palabra);

							sentence.words=frase_aux;
							cout<<" --- Sentence words: "<<sentence.words.at(0)<<endl;
							ps_result = true;
						}
						else
						{
							if ( (config.testType=="minimental")&&(questionNumber==17) ) // para el 'ni sí, ni no, ni pero' del minimental se manda la respuesta siempre, buena o no
							{
								std::vector<string> frase_aux;

								string palabra = "FRASE INCORRECTA";
								frase_aux.push_back(palabra);
								
								sentence.words = frase_aux;
								cout<<" Minimental 17 --- Sentence words: "<<sentence.words.at(0)<<endl;
								ps_result = true;
							}
						}

					}
                    ::CoTaskMemFree(pPhrase);
                }
            }
            break;
        }

        m_pSpeechContext->GetEvents(1, &curEvent, &fetched);
    }


		if(ps_result){
			cout << "Intentando publicar..." << endl;
		try
		{
			asrPrx->newSentenceAvailable(sentence);
			cout << "Se ha publicado bien" << endl;
			for(int i=0; i<sentence.words.size(); i++){
				cout<<sentence.words[i]<<endl;
			}
		}catch(Ice::Exception e)
		{
			cout << "ERRORRR" << endl;
		}
	}

    return ps_result;
}
bool WinKinectBasics::ValidateFaceBoxAndPoints(const RectI* pFaceBox, const PointF* pFacePoints)
{
    bool isFaceValid = false;

    if (pFaceBox != nullptr)
    {
		

        INT32 width = pFaceBox->Right - pFaceBox->Left;
        INT32 height = pFaceBox->Bottom - pFaceBox->Top;

        // check if we have a valid rectangle within the bounds of the screen space
        isFaceValid = width > 0 && 
            height > 0 && 
			pFaceBox->Right <= cColorWidth && 
			pFaceBox->Bottom <= cColorHeight;

        if (isFaceValid)
        {
            for (int i = 0; i < FacePointType::FacePointType_Count; i++)
            {
                // check if we have a valid face point within the bounds of the screen space                        
                bool isFacePointValid = pFacePoints[i].X > 0.0f &&
                    pFacePoints[i].Y > 0.0f &&
					pFacePoints[i].X < cColorWidth &&
					pFacePoints[i].Y < cColorHeight;

                if (!isFacePointValid)
                {
                    isFaceValid = false;
                    break;
                }
            }
        }
    }

    return isFaceValid;
}
