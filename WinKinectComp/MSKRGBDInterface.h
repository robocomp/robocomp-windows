#pragma once

#include <Ice/Ice.h>
#include <MSK.h>
#include "WinKinect.h"



class MSKRGBDInterface: public RoboCompMSKRGBD::MSKRGBD 
{
public:

	WinKinectBasics *mykinectbasics;

	MSKRGBDInterface(WinKinectBasics *kinbasics);
	~MSKRGBDInterface(void);

	virtual void getRGBImage(RoboCompMSKRGBD::TRGBImage &RGBImage,const Ice::Current&);
	virtual void getDepthImage(RoboCompMSKRGBD::TDepthImage &depthImage,const Ice::Current&);

};

