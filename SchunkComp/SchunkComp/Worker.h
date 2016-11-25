#pragma once

#include <Windows.h>
#include <JointMotor.h>
#include <string>

#include <stdint.h>

#include <Ice/Ice.h>
#include <Ice/Application.h>


typedef int(__stdcall * Cube_openDevice)     (int* piDeviceId, const char* acInitString);
typedef int(__stdcall * Cube_softStopModule) (int   iDeviceId, int iModuleId);
typedef int(__stdcall * Cube_movePos)        (int   iDeviceId, int iModuleId, float fPos);
typedef int(__stdcall * Cube_getVel)         (int   iDeviceId, int iModuleId, float* pfVel);
typedef int(__stdcall * Cube_getPos)         (int   iDeviceId, int iModuleId, float* pfVel);
typedef int(__stdcall * Cube_resetModule)    (int   iDeviceId, int iModuleId);
typedef int(__stdcall * Cube_getMaxPos)      (int   iDeviceId, int iModuleId, float* pfVel);
typedef int(__stdcall * Cube_getMinPos)      (int   iDeviceId, int iModuleId, float* pfVel);
typedef int(__stdcall * Cube_getMaxVel)      (int   iDeviceId, int iModuleId, float* pfVel);
typedef int(__stdcall * Cube_getMaxAcc)      (int   iDeviceId, int iModuleId, float* pfAcc);
typedef int(__stdcall * Cube_setMaxPos)      (int   iDeviceId, int iModuleId, float pfVel);
typedef int(__stdcall * Cube_setMinPos)      (int   iDeviceId, int iModuleId, float pfVel);
typedef int(__stdcall * Cube_setMaxVel)      (int   iDeviceId, int iModuleId, float pfVel);
typedef int(__stdcall * Cube_setMaxAcc)      (int   iDeviceId, int iModuleId, float pfAcc);
typedef int(__stdcall * Cube_moveStep)       (int   iDeviceId, int iModuleId, float fPos, unsigned short uiTime);
typedef int(__stdcall * Cube_moveRamp)       (int   iDeviceId, int iModuleId, float fPos, float pfVel, float pfAcc);
typedef int(__stdcall * Cube_moveVel)        (int   iDeviceId, int iModuleId, float fVel);
typedef int(__stdcall * Cube_softStopModule) (int   iDeviceId, int iModuleId);
typedef int(__stdcall * Cube_softStopAll)	 (int   iDeviceId);

#define DEVICE "ESD:0,1000"

#include "Servo.h"
class Worker
{
public:
	Worker(RoboCompJointMotor::JointMotorPublishPrx proxy, Ice::PropertiesPtr props);
	~Worker();

	void start()
	{
		if (CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)static_entry, this, 0, NULL) == NULL)
			printf("can't create worker thread\n");
	}

	void getState(const std::string & motor, RoboCompJointMotor::MotorState & state);
	void setPosition(const RoboCompJointMotor::MotorGoalPosition & goalPosition);
	void setVelocity(const RoboCompJointMotor::MotorGoalVelocity & goalVelocity);
	void setSyncVelocity(const RoboCompJointMotor::MotorGoalVelocityList & goalVelList);
	void setSyncPosition(const RoboCompJointMotor::MotorGoalPositionList & goalPosList);
	void getMotorParams(const std::string & motor, RoboCompJointMotor::MotorParams & mp);
	RoboCompJointMotor::MotorParamsList getAllMotorParams();
	RoboCompJointMotor::MotorStateMap getAllMotorState();
	RoboCompJointMotor::BusParams getBusParams();
	void setZeroPos(const std::string &motor);
	void setSyncZeroPos();
	void stopAllMotors();
	void stopMotor(const std::string &motor);
	bool checkMotorConnection();
private:
	void compute();
	void run();
	void initializeMotors();
	void checkLimits();
	float truncatePosition(std::string name, float position);
	float truncateVelocity(std::string name, float velocity);

	//PowerCube
	static DWORD static_entry(LPVOID* param)
	{
		Worker *myObj = (Worker*)param;
		myObj->compute();
		return 0;
	}
	

	int Loadm5apiw32(void);
	int UnLoadm5apiw32(void);


	//PowerCube
	HANDLE hwMutex, dtMutex;
	HINSTANCE hFtLibDLL;
	Cube_openDevice openDevice;
	Cube_softStopModule softStopModule;
	Cube_softStopAll softStopAll;
	Cube_getVel getVel;
	Cube_getPos getPos;
	Cube_movePos movePos;
	Cube_resetModule resetModule;
	Cube_setMaxPos setMaxPos;
	Cube_setMinPos setMinPos;
	Cube_setMaxVel setMaxVel;
	Cube_setMaxAcc setMaxAcc;
	Cube_getMaxPos getMaxPos;
	Cube_getMinPos getMinPos;
	Cube_getMaxVel getMaxVel;
	Cube_getMaxAcc getMaxAcc;
	Cube_moveStep moveStep;
	Cube_moveRamp moveRamp;
	Cube_moveVel moveVel;
	//Ice
	RoboCompJointMotor::JointMotorPublishPrx jointmotor_proxy;
	RoboCompJointMotor::MotorStateMap motorStateMap;
	//Parameters
	RoboCompJointMotor::BusParams busParams;
	RoboCompJointMotor::MotorParamsList motorParamsList;
	std::map<std::string, RoboCompJointMotor::MotorParams> mParams;
	//Exceptions
	RoboCompJointMotor::HardwareFailedException hFailed;
	RoboCompJointMotor::UnknownMotorException uFailed;
	//Motors
	std::map<std::string, Servo*> motorsName;
	std::map<int, Servo*> motorsId;
	std::map<std::string, int> name2id;
	int devHandler;

private:
	bool updateInfoMotors();

};

