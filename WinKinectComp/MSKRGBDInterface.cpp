#include "MSKRGBDInterface.h"


MSKRGBDInterface::MSKRGBDInterface(WinKinectBasics *kinbasics)
{

	mykinectbasics=kinbasics;
}


MSKRGBDInterface::~MSKRGBDInterface(void)
{
}


void MSKRGBDInterface::getRGBImage(RoboCompMSKRGBD::TRGBImage &RGBImage,const Ice::Current&){

	RGBImage=mykinectbasics->imageStruct;

}
void  MSKRGBDInterface::getDepthImage(RoboCompMSKRGBD::TDepthImage &depthImage,const Ice::Current&){

	depthImage=mykinectbasics->depthStruct;

}