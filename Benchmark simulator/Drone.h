#pragma once

#include <vector>
#include <queue>
#include "stateClasses.h"
#include "Definitions.h"



struct Measurement {
	int id;
	State icebergState;
};

class Drone{
public:
	int id;

	//Physical properties
	State droneState;
	State shipState;

	Path path;

	std::vector<int> dronesInNetwork;
	bool isInComWithShip() {
		return  find(dronesInNetwork.begin(), dronesInNetwork.end(), -1) != dronesInNetwork.end();	
	}
	std::vector<IcebergData> currentIcebergData;
	int currentIceberg;

	void updateIcebergData(std::vector<IcebergData>* icebergData);
	void updateIcebergData();

	bool isUpToDate = false;

	State state() { return droneState; }

	int timeoutTimer = TIMEOUTMAX;
	bool timerStartet =false;

	void run(bool incrementTime);

	Drone();
};

bool compareByPriority(const IcebergData* lhs, const IcebergData* rhs);