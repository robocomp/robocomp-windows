#include "JointMotorI.h"


JointMotorI::JointMotorI(Worker *_worker)
{
	worker = _worker;
}


JointMotorI::~JointMotorI()
{
	// Free component resources here
}

// Component functions, implementation

void JointMotorI::setPosition(const MotorGoalPosition & goalPos, const Ice::Current&)
{
	try
	{
		worker->setPosition(goalPos);
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch (RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}

void JointMotorI::setSyncPosition(const MotorGoalPositionList & goalPosList, const Ice::Current &)
{
	try
	{
		worker->setSyncPosition(goalPosList);
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}

void JointMotorI::setSyncVelocity(const RoboCompJointMotor::MotorGoalVelocityList& mVelocity, const Ice::Current&)
{
	try
	{
		worker->setSyncVelocity(mVelocity);
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch (RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}
void JointMotorI::setVelocity(const RoboCompJointMotor::MotorGoalVelocity& velocity, const Ice::Current&)
{
	try
	{
		worker->setVelocity(velocity);
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch (RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}

MotorParams JointMotorI::getMotorParams(const ::std::string& motor, const Ice::Current &)
{
	MotorParams mp;
	try
	{
		worker->getMotorParams(motor, mp);
		return mp;
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}

BusParams JointMotorI::getBusParams(const Ice::Current &)
{
	return worker->getBusParams();
}

MotorState JointMotorI::getMotorState(const ::std::string & motor, const Ice::Current &)
{
	MotorState state;
	try
	{
		worker->getState(motor, state);
		return state;
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}

MotorStateMap JointMotorI::getMotorStateMap(const MotorList &motorList, const ::Ice::Current&)
{
	MotorStateMap stateMap;
	MotorState state;
	for(auto motor : motorList)
	{
		try
		{
			worker->getState(motor, state);
			stateMap[motor] = state;
		}
		catch (RoboCompJointMotor::UnknownMotorException & ex)
		{
			throw ex;
		}
	}
	return stateMap;
}

MotorParamsList JointMotorI::getAllMotorParams(const ::Ice::Current&)
{
	MotorParamsList list;
	list = worker->getAllMotorParams();
	if (list.empty() == false)
	{
		return list;
	}
	else
	{
		printf("JointMotor::getAllMotorParams - Empty List\n");
		RoboCompJointMotor::UnknownMotorException ex("JointMotor::getAllMotorParams - Empty List");
		printf("%s:%d  ->  %s\n", __FILE__, __LINE__, __FUNCTION__);
		throw ex;
	}
}

void JointMotorI::getAllMotorState(MotorStateMap &mstateMap, const ::Ice::Current&)
{
	mstateMap = worker->getAllMotorState();
}
void JointMotorI::setZeroPos(const std::string &motor, const ::Ice::Current&)
{
	try
	{
		worker->setZeroPos(motor);
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}
void JointMotorI::setSyncZeroPos(const ::Ice::Current&)
{
	try
	{
		worker->setSyncZeroPos();
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}

void JointMotorI::stopAllMotors(const Ice::Current&)
{
	try
	{
		worker->stopAllMotors();
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}
void JointMotorI::stopMotor(const std::string &motor, const Ice::Current&)
{
	try
	{
		worker->stopMotor(motor);
	}
	catch (RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}
