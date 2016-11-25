/*
*    Copyright (C) 2010 by RoboLab - University of Extremadura
*
*    This file is part of RoboComp
*
*    RoboComp is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    RoboComp is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SERVO_H
#define SERVO_H

#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include <stdint.h>

#include <JointMotor.h>

#define ERR_POSITION_STEPS 15
//#define RAW_DEGREES_RANGE 180
//Total range in radians assuming equal motors
//#define RAW_RADIANS_RANGE ( RAW_DEGREES_RANGE * M_PI / 180 )
//~ #define RAW_STEPS_RANGE 310000.

class Servo
{
public:
	struct TMotorData
	{
		std::string name;
		uint8_t busId;
		float maxPos;
		float minPos;
		float zeroPos;
		int temperature;
		float targetVelocityRads;
		float currentVelocityRads;
		float antPosRads;		 //Steps
		int currentPos;  //Steps
		float currentPosRads;
		float maxVelocityRads;
		//int PWMFreqDiv;
		//int deadBand;
		int currentPower;
		//int P;
		//int D;
		//int I;
		//bool reverseSeek;
		//int maxVelocity;
		//bool NORMAL_ANGLES;
		bool isMoving;
	};


public:
	Servo(RoboCompJointMotor::MotorParams params);
	~Servo();

	TMotorData data;

	//Inicializaci�n de rangos. La llamada a este m�todo es obligatoria tras la creaci�n del servo, para que los m�todos de conversi�n funcionen correctamente.
	void setMotorRanges(float step_range, float degrees_range, float speed_step_range, float max_speed_rads);

	void readIniData();
	bool pendAct;      //Pendiente para actualizar posicion
	bool newCommand;   //Petici�n de movimiento
	void setPendiente(int pos);
	bool pendiente();
	float steps2Rads(int p);
	int rads2Steps(float r);
	int radsPerSec2Steps(float rs);

	RoboCompJointMotor::MotorParams params;

	float RAW_STEPS_RANGE;
	float RAW_DEGREES_RANGE;
	float RAW_RADIANS_RANGE;
	float RAW_SPEED_STEPS_RANGE;
	float MAX_SPEED_RADS;


};

#endif // SERVO_H


