#pragma once

#include <Ice/Ice.h>
#include <MSK.h>
#include "WinKinect.h"


class MSKASRConfigInterface: public RoboCompMSKASR::MSKASRConfig
{
public:

	WinKinectBasics *mykinectbasics;

	MSKASRConfigInterface(WinKinectBasics *kinbasics);
	~MSKASRConfigInterface(void);

	virtual void  setTestConfig(const RoboCompMSKASR::TestConfig &config,const Ice::Current&);
	virtual void  setTestQuestion(int questionNumber,const Ice::Current&);
	
};