#include "Worker.h"


int Worker::Loadm5apiw32(void)
{
	int   dwStatus;

	dwStatus = 1;
	hFtLibDLL = LoadLibrary(L"m5apiw32.dll");
	if (hFtLibDLL != NULL)
	{
		openDevice = (Cube_openDevice)GetProcAddress(hFtLibDLL, "PCube_openDevice");
		softStopModule = (Cube_softStopModule)GetProcAddress(hFtLibDLL, "PCube_softStopModule");
		movePos = (Cube_movePos)GetProcAddress(hFtLibDLL, "PCube_movePos");
		getVel = (Cube_getVel)GetProcAddress(hFtLibDLL, "PCube_getVel");
		getPos = (Cube_getPos)GetProcAddress(hFtLibDLL, "PCube_getPos");
		resetModule = (Cube_resetModule)GetProcAddress(hFtLibDLL, "PCube_resetModule");
		setMaxPos = (Cube_setMaxPos)GetProcAddress(hFtLibDLL, "PCube_setMaxPos");
		setMinPos = (Cube_setMinPos)GetProcAddress(hFtLibDLL, "PCube_setMinPos");
		setMaxVel = (Cube_setMaxVel)GetProcAddress(hFtLibDLL, "PCube_setMaxVel");
		setMaxAcc = (Cube_setMaxAcc)GetProcAddress(hFtLibDLL, "PCube_setMaxAcc");
		getMaxPos = (Cube_getMaxPos)GetProcAddress(hFtLibDLL, "PCube_getMaxPos");
		getMinPos = (Cube_getMinPos)GetProcAddress(hFtLibDLL, "PCube_getMinPos");
		getMaxVel = (Cube_getMaxVel)GetProcAddress(hFtLibDLL, "PCube_getMaxVel");
		getMaxAcc = (Cube_getMaxAcc)GetProcAddress(hFtLibDLL, "PCube_getMaxAcc");
		moveStep = (Cube_moveStep)GetProcAddress(hFtLibDLL, "PCube_moveStep");
		moveRamp = (Cube_moveRamp)GetProcAddress(hFtLibDLL, "PCube_moveRamp");
		moveVel = (Cube_moveVel)GetProcAddress(hFtLibDLL, "PCube_moveVel");
		softStopModule = (Cube_softStopModule)GetProcAddress(hFtLibDLL, "PCube_softStopModule");
		softStopAll = (Cube_softStopAll)GetProcAddress(hFtLibDLL, "PCube_softStopAll");
	}
	else
	{
		FreeLibrary(hFtLibDLL);
		dwStatus = 0;
	}

	return dwStatus;
}


int Worker::UnLoadm5apiw32(void)
{
	int  dwStatus;
	dwStatus = FreeLibrary(hFtLibDLL);
	hFtLibDLL = NULL;
	return (dwStatus);
}


Worker::Worker(RoboCompJointMotor::JointMotorPublishPrx proxy, Ice::PropertiesPtr props)
{
	jointmotor_proxy = proxy;

	if ((hwMutex = CreateMutex(NULL, FALSE, NULL)) == NULL)
	{
		printf("Can't create hwMutex: error %d\n", GetLastError());
		exit(-1);
	}
	if ((dtMutex = CreateMutex(NULL, FALSE, NULL)) == NULL)
	{
		printf("Can't create dtMutex: error %d\n", GetLastError());
		exit(-1);
	}
	Loadm5apiw32();



	busParams.numMotors = props->getPropertyAsIntWithDefault("Schunk.NumMotors", 0);
	printf("\nbusParams.numMotors: %d\n", busParams.numMotors);
	if (busParams.numMotors == 0 || busParams.numMotors>100)
	{
		exit(-1);
	}

	busParams.baudRate = props->getPropertyAsIntWithDefault("Schunk.BaudRate", 0);
	printf("busParams.baudRate: %d\n", busParams.baudRate);
	if (busParams.baudRate == 0)
	{
		exit(-1);
	}

	busParams.basicPeriod = props->getPropertyAsIntWithDefault("Schunk.BasicPeriod", 0);
	printf("busParams.basicPeriod: %d\n", busParams.basicPeriod);
	if (busParams.basicPeriod == 0)
	{
		exit(-1);
	}


	for (int i = 0; i<busParams.numMotors; i++)
	{
		printf("Motor %d\n", i);
		std::string s = std::to_string(i);
		RoboCompJointMotor::MotorParams mpar;
		mpar.name = props->getProperty("Schunk.Params_" + s + ".name");
		printf("\tname:    %s\n", mpar.name.c_str());
		mpar.busId = props->getPropertyAsInt("Schunk.Params_" + s + ".busId");
		printf("\tbusId:   %d\n", mpar.busId);
		mpar.invertedSign = props->getProperty("Schunk.Params_" + s + ".invertedSign") == "true";
		printf("\tinvert:  %d\n", mpar.invertedSign);
		mpar.zeroPos = (float)atof(props->getProperty("Schunk.Params_" + s + ".zeroPos").c_str());
		printf("\tzeroPos: %f\n", mpar.zeroPos);
		mpar.minPos = mpar.zeroPos + (float)atof(props->getProperty("Schunk.Params_" + s + ".minPos").c_str());
		printf("\tminPos:  %f\n", mpar.minPos);
		mpar.maxPos = mpar.zeroPos + (float)atof(props->getProperty("Schunk.Params_" + s + ".maxPos").c_str())  ;
		printf("\tmaxPos:  %f\n", mpar.maxPos);
		mpar.maxVelocity = (float)atof(props->getProperty("Schunk.Params_" + s + ".maxVelocity").c_str());
		printf("\tmaxVel:  %f\n", mpar.maxVelocity);
		motorParamsList.push_back(mpar);
		mParams[mpar.name] = mpar;
		name2id[mpar.name] = mpar.busId;
	}

	initializeMotors();
	
}


Worker::~Worker()
{
}
// check if motor connecting is working
bool Worker::checkMotorConnection()
{
	printf("Schunk::checkMotorConnection Check motor read positions\n");
	int error = 0;
	for (int i = 0;i < 10;i++)
	{
		if (!updateInfoMotors())
		{
			error++;
		}
	}
	return error <= 4;
}
void Worker::initializeMotors()
{
	int ret;
	printf("Schunk::initializeMotors()\n");
	// Open and initialize the device
	ret = openDevice(&devHandler, DEVICE);
	printf("Schunk::initializeMotors() PCube_openDevice() returned: %d\n", ret);

	if (ret != 0)
	{
		std::string error = "Schunk::initializeMotors() - Device could not be opened. Try to check de init string (" + std::string(DEVICE) + ")";
		throw error;
	}

	///Create servos instances in a QMap indexed by name
	for (int i = 0; i < busParams.numMotors; i++)
	{
		const std::string name = motorParamsList.operator[](i).name;
		motorsName[name] = new Servo(motorParamsList.operator[](i));
		motorsName[name]->setMotorRanges(552000, 180, 0, 0);
	}

	std::cout << "Schunk::initializeMotors() - Motor Map created with " << busParams.numMotors << " motors: " << std::endl;
	for(auto s : motorsName)
	{
		std::cout << "	" + s.second->params.name << std::endl;
	}


	///Initialize motor params
	for (auto s : motorsName)
	{
		//Create motorStateMap
		RoboCompJointMotor::MotorState motor;
		motorStateMap[s.first] = motor;

		//Set params
		Servo::TMotorData &data = s.second->data;
		RoboCompJointMotor::MotorParams &params = s.second->params;

		std::cout << "Schunk::initializeMotors() - Configuration data of motor " << params.name << std::endl;

		int ret = -1;
		float pos = -1;
		long int posInc = -1;

		if ((ret = resetModule(devHandler, params.busId)) != 0)
			std::cout << "	Error: PCube_resetModule " << ret << std::endl;
		else
			std::cout << "	PCube_resetModule OK" << std::endl;

		///Read current position		
		if ((ret = getPos(devHandler, params.busId, &pos)) == 0)
		{
			data.currentPosRads = pos;
			std::cout << "	Current position (rads): " << data.currentPosRads << "\n";
		}
		else
		{
			std::cout << "	Error reading current position (rads): " << ret << std::endl;
		}
		posInc = -1;
		if ((ret = getPos(devHandler, params.busId, &pos)) == 0)
		{
			//FIXME: debe ser en radianes
			data.antPosRads = data.currentPosRads;
			data.currentPos = s.second->rads2Steps(data.currentPosRads);
			std::cout << "	Current position (steps): " << data.currentPos << std::endl;
		}
		else
		{
			std::cout << "	Error reading current position (steps): " << ret << std::endl;

		}

		///Set limits
		pos = -1;
		float auxValue = params.maxPos;
		if (params.invertedSign)
		{
			auxValue = -params.minPos;
		}
		if ((ret = setMaxPos(devHandler, params.busId, auxValue)) == 0)
		{
			std::cout << "	Max position set (rad): " << auxValue << std::endl;
			pos = -1;
			if (getMaxPos(devHandler, params.busId, &pos) == 0)
			{
				std::cout << "	Max position read (rad): " << pos << std::endl;
			}
			else
			{
				std::cout << "	Error reading Max position" << std::endl;
			}
		}
		else
		{
			std::cout << "	Error setting Max Position (steps): " << ret << std::endl;

		}
		pos = -1;
		auxValue = params.minPos;
		if (params.invertedSign)
		{
			auxValue = -params.maxPos;
		}
		if ((ret = setMinPos(devHandler, params.busId, auxValue)) == 0)
		{
			std::cout << "	Min position set (rad): " << auxValue << std::endl;
			pos = -1;
			if (getMinPos(devHandler, params.busId, &pos) == 0)
			{
				std::cout << "	Min position read (rad): " << pos << std::endl;
			}
			else
			{
				std::cout << "	Fail reading Min position" << std::endl;
			}
		}
		else
		{
			std::cout << "	Error setting Min Position (rad): " << ret << std::endl;

		}

		///set servos to maximum speed
		if ((ret = setMaxVel(devHandler, params.busId, params.maxVelocity)) == 0)
		{
			std::cout << "	Max Velocity set (rad): " << params.maxVelocity << std::endl;
			pos = -1;
			if (getMaxVel(devHandler, params.busId, &pos) == 0)
			{
				std::cout << "	Max Velocity read (rad): " << pos << std::endl;
			}
			else
			{
				std::cout << "	Fail reading Max. Velocity" << std::endl;
			}
		}
		else
		{
			std::cout << "	Error setting Max. Velocity (rad): " << ret << std::endl;

		}
		///set servos to maximum acceleration
//		if ((ret = setMaxAcc(devHandler, params.busId, 0.1)) == 0)
		{
			std::cout << "	Max Acceleration set (rad): " << 0.2 << std::endl;
			pos = -1;
			if (getMaxAcc(devHandler, params.busId, &pos) == 0)
			{
				std::cout << "	Max Acceleration read (rad): " << pos << std::endl;
			}
			else
			{
				std::cout << "	Fail reading Max. Acceleration" << std::endl;
			}
		}
//		else
		{
			std::cout << "	Error setting Max. Acceleration (rad): " << ret << std::endl;

		}
	}
}

void Worker::compute()
{
	int sleepTime = busParams.basicPeriod;
	while (true)
	{
		if (WaitForSingleObject(hwMutex, INFINITE) == WAIT_OBJECT_0)
		{
			if (WaitForSingleObject(dtMutex, INFINITE) == WAIT_OBJECT_0)
			{
				{{
					updateInfoMotors();
					checkLimits();
				}}
				ReleaseMutex(dtMutex);
			}
			ReleaseMutex(hwMutex);
		}
		Sleep(sleepTime);
	}
}
// return false is motor position could not be retrieve
bool Worker::updateInfoMotors()
{
	float rpos;
	bool isMotorMoving;
	float pvel = -1;
	int ret = -1;
	bool error = true;
	for( auto s : motorsName)
	{
		Servo::TMotorData &data = s.second->data;
		pvel = -1;
		ret = -1;
		try
		{
			if ((ret = getVel(devHandler, s.second->params.busId, &pvel)) != 0)
			{
				printf("JointMotor::Schunk::Update: ERROR PCube_getVel no se pudo obtener la velocidad del motor %d: %d\n", s.second->params.busId, ret);
				error = false;
				continue;
			}
			isMotorMoving = (fabs(pvel) > 0.001)?true:false;

			rpos = 0;
			if (( ret = getPos(devHandler, s.second->params.busId, &rpos)) != 0)
			{
				printf("JointMotor::Schunk::Update: ERROR PCube_getPos no se pudo obtener la posición del motor %d: %d\n", s.second->params.busId, ret);
				error = false;
				continue;
			}
			else
			{
				rpos -= data.zeroPos;
				if (mParams[data.name].invertedSign == true)
					rpos = -rpos;

				data.antPosRads = data.currentPosRads;
				data.currentPos = (int)rpos;
				data.currentPosRads = rpos;
				data.isMoving = isMotorMoving;

				//Copy info to motorStateMap
				motorStateMap[s.first].pos = data.currentPosRads;
				motorStateMap[s.first].v = (int)data.currentVelocityRads;
				motorStateMap[s.first].p = data.currentPos;
				motorStateMap[s.first].isMoving = data.isMoving;
				motorStateMap[s.first].vel = data.currentVelocityRads;
			}
		}
		catch (std::string &s) { throw s; }
	}
	//publish 	
	if (!error)
	{
		try
		{
			//std::cout << " publish motor states" << std::endl;
			//jointmotor_proxy->motorStates(motorStateMap);
		}
		catch (...)
		{
			std::cout << "Error publishing motorState " << std::endl;
		}
	}
	return error;
}

//Check positions limit
void Worker::checkLimits()
{
	for (auto s : motorsName)
	{
		Servo::TMotorData &data = s.second->data;
		if (data.isMoving ) 
		{ 
			float realPos = (data.currentPosRads + data.zeroPos);
			float antPos = (data.antPosRads + data.zeroPos);
			if ((fabs(data.maxPos - realPos) < 0.05) && (data.maxPos - realPos) > 0 && (fabs(data.maxPos - realPos) < fabs(data.maxPos - antPos)))
			{
				moveVel(this->devHandler, data.busId, 0.f);
				std::cout << "Motor: " << s.first << " to close ( " << data.currentPosRads << " ) to limit ( " << data.maxPos << " ) ==> Stopped" << std::endl;
			}
			else if ((fabs(data.minPos - realPos) < 0.05) && (data.minPos - realPos) > 0 && (fabs(data.minPos - realPos) < fabs(data.minPos - antPos)))
			{
				moveVel(this->devHandler, data.busId, 0.f);
				std::cout << "Motor: " << s.first << " to close ( " << data.currentPosRads << " ) to limit ( " << data.minPos << " ) ==> Stopped" << std::endl;
			}
		}
	}
}

void Worker::getState(const std::string & motor, RoboCompJointMotor::MotorState & state)
{
	if (WaitForSingleObject(dtMutex, INFINITE) != WAIT_OBJECT_0) return;
	//printf("%s:%d  ->  %s\n", __FILE__, __LINE__, __FUNCTION__);
	if ( motorStateMap.find(motor) != motorStateMap.end() )
	{
		state = motorStateMap[motor];
	}
	/*
	std::string name = motor;
	if (mParams.find(name) != mParams.end())
	{
		state.pos = motorsName[name]->data.currentPosRads;
		state.v = (int)motorsName[name]->data.currentVelocityRads;
		state.p = motorsName[name]->data.currentPos;
		state.temperature = motorsName[name]->data.temperature;
		state.isMoving = motorsName[name]->data.isMoving;
		state.vel = motorsName[name]->data.currentVelocityRads;
	}*/
	else
	{
		uFailed.what = std::string("Exception: SchunkComp::getMotorState:: Unknown motor name") + motor;
		ReleaseMutex(dtMutex);
		throw uFailed;
	}
	ReleaseMutex(dtMutex);
}

void Worker::setPosition(const RoboCompJointMotor::MotorGoalPosition &goalPosition)
{
	int m = WaitForSingleObject(hwMutex, INFINITE);
	if (m != WAIT_OBJECT_0)
	{
		printf("Can't acquire hardware mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
		return;
	}

	std::string name = goalPosition.name;
	if (mParams.find(name) != mParams.end())
	{
		Servo *servo = motorsName[name];
		int busId = name2id[name];

		float rvel = truncateVelocity(name, goalPosition.maxSpeed);
		float position = truncatePosition(name, goalPosition.position) + servo->data.zeroPos;
		std::cout << "vel " << rvel << " current Pos: " << servo->data.currentPosRads << " newPos: " << position << std::endl;
		std::cout << "zero Pos "<< servo->data.zeroPos <<" setting position (rads) " << position << " from " << servo->data.currentPosRads << " motor(" << name << ") busId(" << busId << ") vel(" << rvel << ")" << std::endl;
		if (moveRamp(devHandler, busId, position, rvel, 0.45f))
		{
			ReleaseMutex(hwMutex);
			printf("Schunk::setPosition() - Error setting position\n");
			throw std::string("Schunk::setPosition() - Error setting position");
		}
	}
	else
	{
		uFailed.what = std::string("Exception: SchunkComp::setPosition:: Unknown motor name") + goalPosition.name;
		printf("%s\n", uFailed.what.c_str());
		ReleaseMutex(hwMutex);
		throw uFailed;
	}
	ReleaseMutex(hwMutex);
	std::cout << "done:setting position of motor" << name << std::endl;
}

void Worker::setVelocity(const RoboCompJointMotor::MotorGoalVelocity & goalVelocity)
{
	if (WaitForSingleObject(hwMutex, INFINITE) != WAIT_OBJECT_0) return;

	std::string name = goalVelocity.name;
	float velocity;
	if (mParams.find(name) != mParams.end())
	{
		Servo *servo = motorsName[name];
		int busId = name2id[name];
		velocity = truncateVelocity(name, goalVelocity.velocity);
		std::cout <<"Set velocity: " << velocity << std::endl;
		if (moveVel(this->devHandler, busId, velocity) != 0)
		{
			ReleaseMutex(hwMutex);
			throw std::string("Schunk::setVelocity() - Error setting Velocity");
		}
	}
	else
	{
		uFailed.what = std::string("Exception: SchunkComp::setVelocity:: Unknown motor name") + goalVelocity.name;
		ReleaseMutex(hwMutex);
		throw uFailed;
	}
	ReleaseMutex(hwMutex);
	std::cout << "done:setting velocity of motor" << name;
}

void Worker::setSyncVelocity(const RoboCompJointMotor::MotorGoalVelocityList & goalVelList)
{
	int m = WaitForSingleObject(dtMutex, INFINITE);
	if (m != WAIT_OBJECT_0)
	{
		printf("Can't acquire data mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
		return;
	}
	bool correct = true;
	uint32_t nGoals = goalVelList.size();
	int *ids = new int[nGoals];
	float *velocities = new float[nGoals];
	for (uint32_t i = 0; i<nGoals; i++)
	{
		std::string name = goalVelList[i].name;
		if (motorsName.find(name) == motorsName.end())
		{
			correct = false;
			break;
		}
		else
		{
			Servo *servo = motorsName[name];
			ids[i] = name2id[name];
			velocities[i] = truncateVelocity(name, goalVelList[i].velocity);
			std::cout << " setting velocity (rads) " << velocities[i] << " motor(" << name << ") busId(" << ids[i] << ")" << std::endl;
		}
	}
	ReleaseMutex(dtMutex);
	if (correct)
	{
		m = WaitForSingleObject(hwMutex, INFINITE);
		if (m != WAIT_OBJECT_0)
		{
			printf("Can't acquire hardware mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
			return;
		}
		for (uint32_t i = 0; i < nGoals; i++)
		{
			if (moveVel(this->devHandler, ids[i], velocities[i]) != 0)
			{
				ReleaseMutex(hwMutex);
				printf("Schunk::setSyncVelocity() - Error setting velocity, motor: %i\n", ids[i]);
				delete ids;
				delete velocities;
				throw std::string("Schunk::setSyncVelocity() - Error setting velocity");
			}
		}
		ReleaseMutex(hwMutex);
	}
	else
	{
		uFailed.what = std::string("Exception: SchunkComp::setSyncVelocity:: Unknown motor name");
		delete ids;
		delete velocities;
		throw uFailed;
	}
	delete ids;
	delete velocities;
	std::cout << "done:setting sync velocity of motors" << std::endl;
}

void Worker::setSyncPosition(const RoboCompJointMotor::MotorGoalPositionList & goalPosList)
{
	for (auto p : goalPosList)
	{
		setPosition(p);
	}
/*
	std::cout << "set syn position" << std::endl;
	int m = WaitForSingleObject(dtMutex, INFINITE);
	if (m != WAIT_OBJECT_0)
	{
		printf("Can't acquire data mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
		return;
	}
	bool correct = true;
	uint32_t nGoals = goalPosList.size();
	int *ids = new int[nGoals];
	float *positions = new float[nGoals];
	float *velocities = new float[nGoals];
	
	for (uint32_t i=0; i<nGoals; i++)
	{
		std::string name = goalPosList[i].name;
		if (motorsName.find(name) == motorsName.end())
		{
			correct = false;
			break;
		}
		else
		{
			Servo *servo = motorsName[name];
			ids[i] = name2id[name];
			positions[i] = truncatePosition(name, goalPosList[i].position) + servo->data.zeroPos;
			velocities[i] = (float)truncateVelocity(name, goalPosList[i].maxSpeed);
			std::cout << " setting position (rads) " << positions[i] << " from " << servo->data.currentPosRads << " motor(" << name << ") busId(" << ids[i] << ") vel(" << velocities[i] << ")" << std::endl;
		}
	}
	ReleaseMutex(dtMutex);
	if (correct)
	{
		m = WaitForSingleObject(hwMutex, INFINITE);
		if (m != WAIT_OBJECT_0)
		{
			printf("Can't acquire hardware mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
			return;
		}
		for (uint32_t i = 0; i < nGoals; i++)
		{
			if (moveRamp(devHandler, ids[i], positions[i], velocities[i], 0.45f) ==0)
			{
				ReleaseMutex(hwMutex);
				printf("Schunk::setSyncPosition() - Error setting position, motor: %i\n", ids[i]);
				delete ids;
				delete positions;
				delete velocities;
				throw std::string("Schunk::setSyncPosition() - Error setting position");
			}
		}
		ReleaseMutex(hwMutex);
	}
	else
	{
		uFailed.what = std::string("Exception: SchunkComp::setSyncPosition:: Unknown motor name");
		delete ids;
		delete positions;
		delete velocities;
		throw uFailed;
	}
	delete ids;
	delete positions;
	delete velocities;
	std::cout << "done:setting sync position of motors" << std::endl;
*/

}

void Worker::getMotorParams(const std::string & motor, RoboCompJointMotor::MotorParams & mp)
{
	int m = WaitForSingleObject(dtMutex, INFINITE);
	if (m != WAIT_OBJECT_0)
	{
		printf("Can't acquire data mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
		return;
	}
	std::string name = motor;
	if (mParams.find(name) != mParams.end())
	{
		mp = mParams[name];
	}
	else
	{
		ReleaseMutex(dtMutex);
		uFailed.what = std::string("Exception: SchunkComp::getMotorParams:: Unknown motor name") + motor;
		throw uFailed;
	}
	ReleaseMutex(dtMutex);
}

RoboCompJointMotor::MotorParamsList Worker::getAllMotorParams()
{
	int m = WaitForSingleObject(dtMutex, INFINITE);
	if (m!= WAIT_OBJECT_0)
	{
		printf("Can't acquire data mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
		return RoboCompJointMotor::MotorParamsList();
	}
	RoboCompJointMotor::MotorParamsList ret = motorParamsList;
	ReleaseMutex(dtMutex);
	return ret;
}

RoboCompJointMotor::MotorStateMap Worker::getAllMotorState()
{
	if (WaitForSingleObject(dtMutex, INFINITE) != WAIT_OBJECT_0)
	{
		printf("Can't acquire data mutex %s:%d  ->  %s\n", __FILE__, __LINE__, __FUNCTION__);
		return RoboCompJointMotor::MotorStateMap();
	}
	RoboCompJointMotor::MotorStateMap mstateMap;
	mstateMap = motorStateMap;
/*	RoboCompJointMotor::MotorState state;
	for(auto ss : motorsName)
	{
		const auto s = ss.second;
		state.pos = s->data.currentPosRads;
		state.v = (int)s->data.currentVelocityRads;
		state.p = s->data.currentPos;
		state.isMoving = s->data.isMoving;
		state.temperature = s->data.temperature;
		mstateMap[s->params.name] = state;
	}*/
	ReleaseMutex(dtMutex);
	return mstateMap;
}

RoboCompJointMotor::BusParams Worker::getBusParams()
{
	if (WaitForSingleObject(dtMutex, INFINITE) != WAIT_OBJECT_0) return RoboCompJointMotor::BusParams();
	RoboCompJointMotor::BusParams ret = busParams;
	ReleaseMutex(dtMutex);
	return ret;
}

void Worker::setZeroPos(const std::string &motor)
{
	RoboCompJointMotor::MotorGoalPosition mpos;
	mpos.maxSpeed = mParams[motor].maxVelocity;
	mpos.name = motor;
	mpos.position = 0;
	setPosition(mpos);
}

void Worker::setSyncZeroPos()
{
	RoboCompJointMotor::MotorGoalPositionList goalPosList;
	for (auto s : motorsName)
	{
		RoboCompJointMotor::MotorGoalPosition mgoal;
		mgoal.maxSpeed = mParams[s.first].maxVelocity;
		mgoal.name = s.first;
		mgoal.position = 0;
		goalPosList.push_back(mgoal);
	}
	setSyncPosition(goalPosList);
}


void Worker::stopAllMotors()
{
	printf("%s:%d  ->  %s\n", __FILE__, __LINE__, __FUNCTION__);
	int m = WaitForSingleObject(hwMutex, INFINITE);
	if (m != WAIT_OBJECT_0)
	{
		printf("Can't acquire hardware mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
		return;
	}
	if (softStopAll(devHandler) != 0)
	{
		std::cout << "	Error: PCube_softStopAll " << std::endl;
		ReleaseMutex(hwMutex);
		throw std::string("Schunk::stopAllMotors() - Error stopping motor");
	}
}

void Worker::stopMotor(const std::string &motor)
{
	if (mParams.find(motor) != mParams.end())
	{
		printf("%s:%d  ->  %s\n", __FILE__, __LINE__, __FUNCTION__);
		int m = WaitForSingleObject(hwMutex, INFINITE);
		if (m != WAIT_OBJECT_0)
		{
			printf("Can't acquire hardware mutex (%d) %s:%d  ->  %s\n", m, __FILE__, __LINE__, __FUNCTION__);
			return;
		}
		int busId = name2id[motor];
		if (softStopModule(devHandler, busId) != 0)
		{
			std::cout << "	Error: PCube_SoftStopModule " << motor << std::endl;
			ReleaseMutex(hwMutex);
			throw std::string("Schunk::stopMotor() - Error stopping motor");
		}
	}
	else
	{
		uFailed.what = std::string("Exception: SchunkComp::stopMotor:: Unknown motor name") + motor;
		ReleaseMutex(hwMutex);
		throw uFailed;
	}
	ReleaseMutex(hwMutex);
	std::cout << "done:Stop motor: " << motor;
}

float Worker::truncatePosition(std::string name, float position)
{
	float const DELTA = 0.05f;
	float p = position;
//	std::cout << "truncate " << name << " " << mParams[name].invertedSign <<" " << position<<"max "<<mParams[name].maxPos<<" " <<mParams[name].minPos <<std::endl;
	if (p > mParams[name].maxPos - DELTA)
		p = mParams[name].maxPos - DELTA;
	else if (p < mParams[name].minPos + DELTA)
		p = mParams[name].minPos + DELTA;
	if (mParams[name].invertedSign == true)
		p = -position;
//	std::cout << " final p " << p << std::endl;
	return p;
}
float Worker::truncateVelocity(std::string name, float velocity)
{
	float v = velocity;
	if (mParams[name].maxVelocity < fabs(velocity))
	{
		v = mParams[name].maxVelocity;
		if (velocity < 0)
			v = -mParams[name].maxVelocity;
	}
	return v;
}


