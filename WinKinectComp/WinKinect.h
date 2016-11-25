//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once


#include "MSK.h"
#include <ctime>
#include <string>
#include <sstream>
#include <fstream>
#include <Windows.h>
#include <minmax.h>
//#include <gdiplus.h>
#include <shlobj.h>

#include <devicetopology.h>
#include <Ice/Ice.h>
#include <IceStorm\IceStorm.h>
#include <Functiondiscoverykeys_devpkey.h>
#include <condition_variable>

#include <stdio.h>
#include <wchar.h>


// For speech APIs
// NOTE: To ensure that application compiles and links against correct SAPI versions (from Microsoft Speech
//       SDK), VC++ include and library paths should be configured to list appropriate paths within Microsoft
//       Speech SDK installation directory before listing the default system include and library directories,
//       which might contain a version of SAPI that is not appropriate for use together with Kinect sensor.
#include <sapi.h>
__pragma(warning(push))
__pragma(warning(disable:6385 6001)) // Suppress warnings in public SDK header
#include <sphelper.h>
__pragma(warning(pop))

#include "stdafx.h"
#include "KinectAudioStream.h"

#define MAXBUFF 256
#define ELAPSED_TIME_4STORE 0 //milliseconds
#define MAX_THREADS 1

#define M_PI       3.14159265358979323846

// Number of milliseconds of acceptable lag between live sound being produced and recording operation.
const int TargetLatency = 20;
static std::mutex mtx;
static std::condition_variable cv;
static bool ready=false;

// define the face frame features required to be computed by this application
static const DWORD c_FaceFrameFeatures = 
    FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
    | FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
    | FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
    | FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
    | FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed;
 
static const double c_FaceRotationIncrementInDegrees = 5.0f;

//const bool storin_ssd = false;

//  Header for a WAV file - we define a structure describing the first few fields in the header for convenience.
struct WAVEHEADER
{
    DWORD   dwRiff;                     // "RIFF"
    DWORD   dwSize;                     // Size
    DWORD   dwWave;                     // "WAVE"
    DWORD   dwFmt;                      // "fmt "
    DWORD   dwFmtSize;                  // Wave Format Size
};


//  Static RIFF header, we'll append the format to it.
const BYTE WaveHeaderTemplate[] = 
{
    'R',   'I',   'F',   'F',  0x00,  0x00,  0x00,  0x00, 'W',   'A',   'V',   'E',   'f',   'm',   't',   ' ', 0x00, 0x00, 0x00, 0x00
};

//  Static wave DATA tag.
const BYTE WaveData[] = { 'd', 'a', 't', 'a'};

using namespace std;
//using namespace Gdiplus;

class WinKinectBasics
{
friend class  MSKASRConfigInterface;
  

public:

	static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;
	static const int        cColorWidth  = 1920;
    static const int        cColorHeight = 1080;
    /// <summary>
    /// Constructor
    /// </summary>
    WinKinectBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~WinKinectBasics();


	void initializingData();

	void stoppingCapture();

	bool started, executing;
	int iter_n;



	MSG						msg;

	//audio thread
	HANDLE					hThreadArray[MAX_THREADS]; 
	DWORD					dwThreadIdArray[MAX_THREADS];
	CONDITION_VARIABLE		stop_capturing_audio;


   // HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;

    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;
	CameraSpacePoint*       cameraSpacePoints;
    // Body reader
    IBodyFrameReader*       m_pBodyFrameReader;

    // Direct2D
   // ID2D1Factory*           m_pD2DFactory;

    // Color reader
    IColorFrameReader*      m_pColorFrameReader;

    // Direct2D
    //ImageRenderer*          m_pDrawColor;
    RGBQUAD*                m_pColorRGBX;

	 // Depth reader
    IDepthFrameReader*      m_pDepthFrameReader;


    RGBQUAD*                m_pDepthRGBX;


    // Face sources
    IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];

    // Face readers
    IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];



	// time variables
	time_t rawtime;
	struct tm * timeinfo;
	SYSTEMTIME systime;
	WORD millis, previous_millis;
	 


	//output folder
	

	unsigned long n_iter;

	// tracking skeleton
	bool tracking; 

	 D2D1_POINT_2F jointPoints[JointType_Count];
    
    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();
    
	/// <summary>
    /// Handle new color data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    /// </summary>
    void                    ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);

	/// <summary>
    /// Handle new depth data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    /// <param name="nMinDepth">minimum reliable depth</param>
    /// <param name="nMaxDepth">maximum reliable depth</param>
    /// </summary>
    void                    ProcessDepth(INT64 nDepthTime, UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, USHORT nMinDepth, USHORT nMaxDepth);
	 /// <summary>
    /// Handle new body data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="nBodyCount">body data count</param>
    /// <param name="ppBodies">body data in frame</param>
    /// </summary>
    void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies,RGBQUAD* pBuffer,int nWidth, int nHeight);

	   /// <summary>
    /// Processes new face frames
    /// </summary>
    RoboCompMSKFace::DetailedFace                  ProcessFaces(IBody** ppBodies, int iface,bool &valid);
	void										   ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll);


	 /// <summary>
    /// Dispose Direct2d resources 
    /// </summary>
  //  void DiscardDirect2DResources();

	/// <summary>
    /// Converts a body point to screen space
    /// </summary>
    /// <param name="bodyPoint">body point to tranform</param>
    /// <param name="width">width (in pixels) of output buffer</param>
    /// <param name="height">height (in pixels) of output buffer</param>
    /// <returns>point in screen-space</returns>
    D2D1_POINT_2F           BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);

	HRESULT					InitializeSpeech();

	// audio capture

	HRESULT					startAudioThread();

	HRESULT					waitForAudioThread();

	 /// <summary>
    /// Create speech recognizer that will read Kinect audio stream data.
    /// </summary>
    /// <returns>
    /// <para>S_OK on success, otherwise failure code.</para>
    /// </returns>
    HRESULT                 CreateSpeechRecognizer();

    /// <summary>
    /// Load speech recognition grammar into recognizer.
    /// </summary>
    /// <returns>
    /// <para>S_OK on success, otherwise failure code.</para>
    /// </returns>
    HRESULT                 LoadSpeechGrammar();

    /// <summary>
    /// Start recognizing speech asynchronously.
    /// </summary>
    /// <returns>
    /// <para>S_OK on success, otherwise failure code.</para>
    /// </returns>
    HRESULT                 StartSpeechRecognition();

	bool					ProcessSpeech();

	bool					ValidateFaceBoxAndPoints(const RectI* pFaceBox, const PointF* pFacePoints);

	RoboCompMSKBody::MSKBodyEventPrx bodyPrx;
	RoboCompMSKFace::MSKFaceEventPrx facePrx;
	RoboCompMSKASR::MSKASREventPrx asrPrx;
	
	RoboCompMSKRGBD::TRGB imagen;
	RoboCompMSKRGBD::TRGBImage imageStruct;
	RoboCompMSKRGBD::TDepthImage depthStruct;

	RoboCompMSKASR::TestConfig config;
	int questionNumber;
	bool changeConfig;
	bool changeQuestion;

	bool salir;

	LPCWSTR          GrammarFileName;

	 // Handle for sensor notifications
    WAITABLE_HANDLE         m_hSensorNotification;
	 // A single audio beam off the Kinect sensor.
    IAudioBeam*             m_pAudioBeam;

    // An IStream derived from the audio beam, used to read audio samples
    IStream*                m_pAudioStream;

    // Stream for converting 32bit Audio provided by Kinect to 16bit required by speeck
    KinectAudioStream*     m_p16BitAudioStream;


    // Stream given to speech recognition engine
    ISpStream*              m_pSpeechStream;

    // Speech recognizer
    ISpRecognizer*          m_pSpeechRecognizer;

    // Speech recognizer context
    ISpRecoContext*         m_pSpeechContext;

    // Speech grammar
    ISpRecoGrammar*         m_pSpeechGrammar;

    // Event triggered when we detect speech recognition
    HANDLE                  m_hSpeechEvent;

	//Para controlar el lenguaje y el tipo de test
	int				idioma;
	int				test;

	std::mutex mtx_I;
};

