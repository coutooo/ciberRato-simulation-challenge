/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2011 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef _CB_PARAMETERS_
#define _CB_PARAMETERS_

#include <qstring.h>

/*!
	\class cbParameters
	\brief recepient to store global simulation parameters
*/

class cbParameters
{
public:
	cbParameters();
	~cbParameters();

	int toXml(char *buff, int n);

public:
	unsigned int simTime, cycleTime;
	unsigned int keyTime;

	//Noise
	double compassNoise, beaconNoise, obstacleNoise;
	double motorsNoise;
	double gpsLinNoise, gpsDirNoise;

	//Requests
	int nReqPerCycle;
	bool obstacleRequestable, compassRequestable, 
	     beaconRequestable, groundRequestable, collisionRequestable, gpsRequestable;

	//Latencies
	unsigned int obstacleLatency, compassLatency, 
		     beaconLatency, groundLatency, collisionLatency, gpsLatency;

    //Scores
    int returnTimePenalty;
    int arrivalTimePenalty;
    int collisionWallPenalty;
    int collisionRobotPenalty;
    int targetReward;
    int homeReward;
	
	double beaconAperture;

	bool   GPSOn;
	bool   beaconSensorOn;
	bool   scoreSensorOn;

	bool   showActions;
	QString labFilename;
	QString gridFilename;
	int nBeacons;

};
#endif
