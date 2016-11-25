module RoboCompMSKRGBD{

	sequence<byte> TRGB;
	sequence<short> TDepth;

	struct TRGBImage{
		TRGB image;
		int height;
		int width;
	};

	struct TDepthImage{
		TDepth image;
		int height;
		int width;
	};

	interface MSKRGBDEvent{
		
		void newRGBImageAvailable(TRGBImage RGBImage);
		void newDepthImageAvailable(TDepthImage depthImage);
		
	};
	interface MSKRGBD{
		
		void getRGBImage(out TRGBImage RGBImage);
		void getDepthImage(out TDepthImage depthImage);
		
	};
};

module RoboCompMSKBody
{
	
	enum JointType
    {
       SpineBase,
       SpineMid,
       Neck,
       Head,
       ShoulderLeft,
       ElbowLeft,
       WristLeft,
       HandLeft,
       ShoulderRight,
       ElbowRight,
       WristRight,
       HandRight,
       HipLeft,
       KneeLeft,
       AnkleLeft,
       FootLeft,
       HipRight,
       KneeRight,
       AnkleRight,
       FootRight,
       SpineShoulder,
       HandTipLeft,
       ThumbLeft,
       HandTipRight,
       ThumbRight,
    };
	
	
	enum stateType{NoTracking, Tracking};
	enum JointTrackingState {NotTracked,Inferred, Tracked};

	struct SkeletonPoint
	{
		float X;
		float Y;
		float Z;
	};

	struct DepthImagePoint
	{
		int X;
		int Y;
		//distance in milimetres
		int Depth;
	};

	struct ColorImagePoint
	{
		int X;
		int Y;
		byte R;
		byte G;
		byte B;
	};

	struct Joint
	{
		JointTrackingState state;
		SkeletonPoint Position;
	};

	dictionary<JointType, Joint> JointList;

	struct TPerson
	{
        JointList joints;
		SkeletonPoint Position;
		int TrackingId;
		bool trackedState;
		
    	};
	
	dictionary<int,TPerson> PersonList;

	
	interface MSKBodyEvent
	{
		void newMSKBodyEvent(PersonList people,long timestamp);
    };
};


module RoboCompMSKFace
{
	struct Point3D{
		float x;
		float y;
		float z;
	};

	// DetailedFace - structure of the detailedFace (face and eyes data, identifier)
	struct DetailedFace
	{
		Point3D nose;
		Point3D leftEar;
		Point3D rightEar;
		int left;
		int right;
		int top;
		int bottom;
		int identifier;
		bool rightEyeClosed;
		bool leftEyeClosed;
		float yaw;
		float pitch;
		float roll;
		RoboCompMSKRGBD::TRGBImage faceImage;
	};

	// FaceMap - A map storing all faces in the scene
	dictionary<int, DetailedFace> DetailedFaceMap;
	
	interface MSKFaceEvent{
		void newFaceAvailable(DetailedFaceMap face,long timestamp);
	};
	
};

module RoboCompMSKASR{
	sequence <string> WordsRecognized;

	struct TSentence{
		WordsRecognized words;
		int acquisitionHour;
		int acquisitionSecs;
		int acquisitionDay;
		bool blockingCall;
		string grammarUsed;
	};
	struct TestConfig{
		string testType;
		string language;
		string tense;
		string person;
	};

	interface MSKASREvent{
		void  newSentenceAvailable(TSentence sentence);
	};
	interface MSKASRConfig{
		void  setTestConfig(TestConfig config);
		void  setTestQuestion(int questionNumber);
	};

};
  

