#pragma once

#include "JointMotor.h"
#include "Worker.h"


using namespace RoboCompJointMotor;

class JointMotorI : public RoboCompJointMotor::JointMotor
{
public:
	JointMotorI(Worker *_worker);
	~JointMotorI();

	void setPosition(const MotorGoalPosition & goalPos, const Ice::Current& = ::Ice::Current());         // Send servo to position
	void setVelocity(const MotorGoalVelocity & goalVel, const Ice::Current& = ::Ice::Current());         // Sets servo to given velocity
	void setSyncVelocity(const RoboCompJointMotor::MotorGoalVelocityList& mVelocity, const Ice::Current& = Ice::Current());
	void setReferenceVelocity(const MotorGoalVelocity & goalVel, const Ice::Current& = ::Ice::Current());         // Sets servo to given velocity
	void setSyncPosition(const MotorGoalPositionList & goalPosList, const Ice::Current & = ::Ice::Current());
	MotorParams getMotorParams(const ::std::string & motor, const Ice::Current& = ::Ice::Current());
	BusParams getBusParams(const Ice::Current & = ::Ice::Current());
	MotorState getMotorState(const ::std::string & motor, const Ice::Current& = ::Ice::Current());
	MotorStateMap getMotorStateMap(const MotorList & motorList, const ::Ice::Current& = ::Ice::Current());
	MotorParamsList getAllMotorParams(const ::Ice::Current& = ::Ice::Current());
	void getAllMotorState(MotorStateMap & mstateMap, const ::Ice::Current& = ::Ice::Current());
	void setZeroPos(const std::string &motor, const ::Ice::Current& = ::Ice::Current());
	void setSyncZeroPos(const ::Ice::Current& = ::Ice::Current());

	void stopAllMotors(const Ice::Current& = ::Ice::Current());
	void stopMotor(const ::std::string &motor, const Ice::Current& = ::Ice::Current());
private:
	Worker *worker;

};

