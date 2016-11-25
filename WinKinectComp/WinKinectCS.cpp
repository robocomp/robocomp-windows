#include "WinKinectCS.h"


//bool salir;

WinKinectBasics kinbasis;

void sighandler(int sig)
{
    cout<< "Signal " << sig << " caught..." << endl;

	kinbasis.salir = true;
}

int
WinKinectCS::run(int argc, char*[])
{

	int status = 0;
	cout << "Dentro de run" << endl;

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	kinbasis.salir = false;

	try{

		//Inicializaciones de Ice
		InitializeIce();
		

	   // busco el tópico en IceStorm con el manejador de tópicos
		IceStorm::TopicPrx topic=CreateTopic("MSKBodyEvent");
		
		// creamos el proxy de publicación
 		Ice::ObjectPrx pub = topic->getPublisher()->ice_oneway();
		kinbasis.bodyPrx=RoboCompMSKBody::MSKBodyEventPrx::uncheckedCast(pub);
    
		if (!kinbasis.bodyPrx)
		{
			 throw "Invalid MSKBodyEvent proxy";
		}



		// busco el tópico en IceStorm con el manejador de tópicos
		IceStorm::TopicPrx topicFace=CreateTopic("MSKFaceEvent");
		
		// creamos el proxy de publicación
 		Ice::ObjectPrx pubFace = topicFace->getPublisher()->ice_oneway();
		kinbasis.facePrx=RoboCompMSKFace::MSKFaceEventPrx::uncheckedCast(pubFace);
    	
		if (!kinbasis.facePrx)
		{
			 throw "Invalid MSKFaceEvent proxy";
		}


		// busco el tópico en IceStorm con el manejador de tópicos
		IceStorm::TopicPrx topicASR=CreateTopic("MSKASREvent");
		
		// creamos el proxy de publicación
 		Ice::ObjectPrx pubAsr = topicASR->getPublisher()->ice_oneway();
		//Ice::ObjectPrx pubAsr = topicASR->getPublisher()->ice_twoway();
		kinbasis.asrPrx=RoboCompMSKASR::MSKASREventPrx::uncheckedCast(pubAsr);
    
		if (!kinbasis.asrPrx)
		{
			 throw "Invalid MSKASREvent proxy";
		}



		 try {
			Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("MSKRGBDAdapter");
			Ice::ObjectPtr object = new MSKRGBDInterface(&kinbasis);
			adapter->add(object, communicator()->stringToIdentity("mskrgbd"));
			adapter->activate();
			//communicator()->waitForShutdown();
			
		} catch (const Ice::Exception& e) {
			cerr << e << endl;
			status = 1;
		} catch (const char* msg) {
			cerr << msg << endl;
			status = 1;
		}
		
		try {
			Ice::ObjectAdapterPtr adapter1 = communicator()->createObjectAdapter("MSKASRConfigAdapter");
			Ice::ObjectPtr object1 = new MSKASRConfigInterface(&kinbasis);
			adapter1->add(object1, communicator()->stringToIdentity("mskasrconfig"));
			adapter1->activate();
			//communicator()->waitForShutdown();
			
		} catch (const Ice::Exception& e) {
			cerr << e << endl;
			status = 1;
		} catch (const char* msg) {
			cerr << msg << endl;
			status = 1;
		}
		
        //////////////////////////////////////////////////////
		
		
		kinbasis.initializingData();
		kinbasis.startAudioThread();


		while(!kinbasis.salir)
		{

				kinbasis.tracking=false;

				if (!kinbasis.m_pBodyFrameReader || !kinbasis.m_pColorFrameReader || !kinbasis.m_pDepthFrameReader)
				{
					return 0;
				}

				INT64 nTime = 0;
				IFrameDescription* pFrameDescription = NULL;
				int nWidth = 0;
				int nHeight = 0;
				ColorImageFormat imageFormat = ColorImageFormat_None;
				UINT nBufferSize = 0;
				RGBQUAD *pBuffer = NULL;
				IBodyFrame* pBodyFrame = NULL;
				IColorFrame* pColorFrame = NULL;
		


				HRESULT hr_color = kinbasis.m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

				if (SUCCEEDED(hr_color))
				{
	
					hr_color = pColorFrame->get_RelativeTime(&nTime);

					if (SUCCEEDED(hr_color))
					{
						hr_color = pColorFrame->get_FrameDescription(&pFrameDescription);
					}

					if (SUCCEEDED(hr_color))
					{
						hr_color = pFrameDescription->get_Width(&nWidth);
					}

					if (SUCCEEDED(hr_color))
					{
						hr_color = pFrameDescription->get_Height(&nHeight);
					}

					if (SUCCEEDED(hr_color))
					{
						hr_color = pColorFrame->get_RawColorImageFormat(&imageFormat);
					}

					if (SUCCEEDED(hr_color))
					{
						if (imageFormat == ColorImageFormat_Bgra)
						{
							hr_color = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
						}
						else if (kinbasis.m_pColorRGBX)
						{
							pBuffer = kinbasis.m_pColorRGBX;
							nBufferSize = kinbasis.cColorWidth * kinbasis.cColorHeight * sizeof(RGBQUAD);
							hr_color = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);            
						}
						else
						{
							hr_color = E_FAIL;
						}
					}

					if (SUCCEEDED(hr_color))
					{
						 kinbasis.ProcessColor(nTime, pBuffer, nWidth, nHeight);
					}

			
					SafeRelease(pFrameDescription);
				}

				SafeRelease(pColorFrame);

				HRESULT hr_body = kinbasis.m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

				if (SUCCEEDED(hr_body))
				{
					INT64 nTime = 0;

					hr_body = pBodyFrame->get_RelativeTime(&nTime);

					IBody* ppBodies[BODY_COUNT] = {0};

					if (SUCCEEDED(hr_body))
					{
						hr_body = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
					}

					if (SUCCEEDED(hr_body))
					{

						kinbasis.ProcessBody(nTime, BODY_COUNT, ppBodies,pBuffer, nWidth, nHeight);
					
					}

					for (int i = 0; i < _countof(ppBodies); ++i)
					{
						SafeRelease(ppBodies[i]);
					}
				}

			 SafeRelease(pBodyFrame);
		

			 IDepthFrame* pDepthFrame = NULL;

			HRESULT hr = kinbasis.m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

			if (SUCCEEDED(hr))
			{
				INT64 nDepthTime = 0;
				IFrameDescription* pDepthFrameDescription = NULL;
				int nDepthWidth = 0;
				int nDepthHeight = 0;
				USHORT nDepthMinReliableDistance = 0;
				USHORT nDepthMaxDistance = 0;
				UINT nDepthBufferSize = 0;
				UINT16 *pDepthBuffer = NULL;

				hr = pDepthFrame->get_RelativeTime(&nTime);

				if (SUCCEEDED(hr))
				{
					hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
				}

				if (SUCCEEDED(hr))
				{
					hr = pDepthFrameDescription->get_Width(&nDepthWidth);
				}

				if (SUCCEEDED(hr))
				{
					hr = pDepthFrameDescription->get_Height(&nDepthHeight);
				}

				if (SUCCEEDED(hr))
				{
					hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
				}

				if (SUCCEEDED(hr))
				{
					// In order to see the full range of depth (including the less reliable far field depth)
					// we are setting nDepthMaxDistance to the extreme potential depth threshold
					nDepthMaxDistance = USHRT_MAX;

					// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
					//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
				}

				if (SUCCEEDED(hr))
				{
					hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);            
				}

				if (SUCCEEDED(hr))
				{
					kinbasis.ProcessDepth(nDepthTime, pDepthBuffer, nDepthHeight, nDepthWidth, nDepthMinReliableDistance, nDepthMaxDistance);
				}

				SafeRelease(pDepthFrameDescription);
			}

			SafeRelease(pDepthFrame);
		
		} //end while
		
	
		/*cout << "publicando dato..." << endl;
		RoboCompMSKBody::PersonList people;
	    bodyPrx->newMSKBodyEvent( people,0);
		cout << "OK!!" << endl;
		system("pause");*/
	} catch (const Ice::Exception& ex) {
		cerr << ex << endl;
		status = 1;
	} catch (const char* msg) {
		cerr << msg << endl;
		status = 1;
	}

	//communicator()->waitForShutdown();


	kinbasis.waitForAudioThread();
	kinbasis.stoppingCapture();
	return status;
		

}


void WinKinectCS::InitializeIce(){


	// Obtener el proxy a IceStorm
	Ice::ObjectPrx obj = communicator()->propertyToProxy("TopicManager.Proxy");
	// Se crea el manejador de tópicos
	 topicManager = IceStorm::TopicManagerPrx::checkedCast(obj);

}


IceStorm::TopicPrx  WinKinectCS::CreateTopic(string topicName){

	IceStorm::TopicPrx topic;
	while (!topic) {
				try {
					topic = topicManager->retrieve(topicName);
				} catch (const IceStorm::NoSuchTopic&) {
					try {
					topic = topicManager->create(topicName);
					} catch (const IceStorm::TopicExists&) {
					// Another client created the topic.
					}
				}
		}
	return topic;
}