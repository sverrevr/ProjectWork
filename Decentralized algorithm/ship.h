#pragma once
#include "stateClasses.h"
#include "Message.h"
#include "Definitions.h"
#include <vector>

struct Ship{
	
	State shipState;
	std::vector<int> dronesInNetwork;
	std::vector<IcebergData> currentIcebergData;
	void updateIcebergData(std::vector<IcebergData>* icebergData);

	void run();
	void updateIcebergData();
};

