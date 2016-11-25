#include "stdafx.h"

#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include "JointMotorI.h"
#include "Worker.h"


int main(int argc, char **argv)
{
	int status = 0;
	Ice::CommunicatorPtr ic;
	try
	{
		ic = Ice::initialize(argc, argv);
	}
	catch (const Ice::Exception& e)
	{
		std::cerr << e << std::endl;
		status = 1;
	}
	catch (const char* msg)
	{
		std::cerr << msg << std::endl;
		status = 1;
	}

/*
	IceStorm::TopicManagerPrx topicManager;
	IceStorm::TopicPrx jointmotor_topic;
	try
	{
		topicManager = IceStorm::TopicManagerPrx::checkedCast(ic->stringToProxy("IceStorm/TopicManager:default -h 158.49.247.100 -p 9999"));
	}
	catch (const Ice::Exception ex)
	{
		std::cout << "Exception connecting to TopicManager" << ex.what() << std::endl;
		exit(-1);
	}
	while (!jointmotor_topic)
	{
		try
		{
			jointmotor_topic = topicManager->retrieve("JointMotor_topic");
		}
		catch (const IceStorm::NoSuchTopic&)
		{
			try
			{
				jointmotor_topic = topicManager->create("JointMotor_topic");
			}
			catch (const IceStorm::TopicExists&) {
				// Another client created the topic.
			}
		}
	}
	Ice::ObjectPrx jointmotor_pub = jointmotor_topic->getPublisher()->ice_oneway();
	*/
	JointMotorPublishPrx jointmotor;
	/*
	jointmotor = JointMotorPublishPrx::uncheckedCast(jointmotor_pub);
	*/

	Worker *worker = new Worker(jointmotor, ic->getProperties());
	if (!worker->checkMotorConnection())
	{
		std::cerr << "Motor API connection not working" << std::endl;
		printf("Motor API connection not working\n");
		exit(-1);
	}
	printf("exit\n");
	worker->start();

	try
	{
		Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("jointmotor", "default -p 10067");
		Ice::ObjectPtr object = new JointMotorI(worker);
		adapter->add(object, ic->stringToIdentity("jointmotor"));
		adapter->activate();
	}
	catch (const Ice::Exception& e)
	{
		std::cerr << e << std::endl;
		status = 1;
	}
	catch (const char* msg)
	{
		std::cerr << msg << std::endl;
		status = 1;
	}

	ic->waitForShutdown();

	if (ic)
	{
		try
		{
			ic->destroy();
		}
		catch (const Ice::Exception& e)
		{
			std::cerr << e << std::endl;
			status = 1;
		}
	}

	return status;
}

