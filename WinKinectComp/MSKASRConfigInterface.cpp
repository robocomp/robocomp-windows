#include "MSKASRConfigInterface.h"

MSKASRConfigInterface::MSKASRConfigInterface(WinKinectBasics *kinbasics){

	mykinectbasics=kinbasics;
}


MSKASRConfigInterface::~MSKASRConfigInterface(void)
{
}

void MSKASRConfigInterface::setTestConfig(const RoboCompMSKASR::TestConfig &config,const Ice::Current&){
	
	mykinectbasics->mtx_I.lock();
	mykinectbasics->config=config;
	mykinectbasics->changeConfig=true;
	mykinectbasics->mtx_I.unlock();
}

void MSKASRConfigInterface::setTestQuestion(int questionNumber,const Ice::Current&){
	mykinectbasics->mtx_I.lock();
	mykinectbasics->questionNumber=questionNumber;
	mykinectbasics->changeQuestion=true;
	mykinectbasics->mtx_I.unlock();
}