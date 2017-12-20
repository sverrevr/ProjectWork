#pragma once

#include <vector>
#include <queue>
#include "stateClasses.h"
#include "Message.h"
#include "Definitions.h"


enum DroneState {state_flying, state_proposingNodes, state_bidding, state_waitingForBids};

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
	std::vector<IcebergData*> currentIcebergData_byPriority;
	void sortIcebergData();
	int findNextFreeIceberg() const;
	int currentIceberg;

	DroneState currentState = state_flying;

	void updateIcebergData(std::vector<IcebergData>* icebergData);
	void removeOwnershipByDrone(int droneId);
	bool calculateBestPath(int icebergId, const Path* basePath, Path* returnPath, double priorityScaler);
	bool calculateBestPathToComNode(int goalIcebergId, const Path* basePath, Path* returnPath, double timepoint, Vector2D dronePos, Vector2D shipPos);
	void setOwnershipFromPath(Path* path);

	bool hasRecievedReMakePath = false;
	bool hasStartedReMakePath = false;
	bool hasSentNode = false;

	void updateIcebergData();

	bool isUpToDate = false;

	State state() { return droneState; }

	int timeoutTimer = TIMEOUTMAX;
	bool timerStartet =false;

	void run(const Message* innputMessage, std::queue<Message*>* messageQue, bool messageQueueEmpty, bool incrementTime);

	void setupRemakePath();

	std::vector<proposedNode> proposedNodes;

	bool limitedToShipRadius = false;

	Drone();
};

bool compareByPriority(const IcebergData* lhs, const IcebergData* rhs);