#pragma once
#include <signal.h>
#include <conio.h>
#include <iostream>
#include "WinKinect.h"
#include "MSKRGBDInterface.h"
#include "MSKASRConfigInterface.h"

using namespace std;

class WinKinectCS : public Ice::Application
{
public:
    virtual int run(int, char*[]);


protected:
	void InitializeIce();
	IceStorm::TopicPrx CreateTopic(string topicName);

private:
	//Ice::CommunicatorPtr ic;
//	Ice::ObjectPrx stormPrx;
	IceStorm::TopicManagerPrx topicManager;


};