#include "Drone.h"
#include "BellmanFord.h"
#include <iostream>
#include <algorithm>
using namespace std;

Drone::Drone() {
	path.reset();
}


void Drone::run(bool incrementTime) {

	//Increment time må skje først siden den resetter stuff som kan bli satt når incrementTime sendes inn
	updateIcebergData();


	//update if we are close to a com node, this is not handled by the thread handler as it only handles in
	//relation to real icebergs
	if (!path.finished(droneState.pos) && path.currentNode().iceberg == PATH_NO_ICEBERG
		&& distance2D(droneState.pos,path.currentNode().possition)<COM_NODE_SLACK){
		path.incrementCurrentGoal();
	}


	
	/*Handle state*/
	Path tempPath;
	tempPath.reset();

	if (incrementTime) {
		Vector2D currentPosGoal = shipState.pos; //default location

		if (!path.finished(droneState.pos)) {
			currentPosGoal = path.currentNode().possition;
		}
		else if (isInComWithShip() && isUpToDate) {
			path.currentGoal = 0;
			//if we are not in com with ship, fly towards the ship untill we are
		}

		Vector2D vectorToGoal = currentPosGoal - droneState.pos;
		normalize(&vectorToGoal);
		droneState.velocity = vectorToGoal* DRONE_MOVEMENT_SPEED;
	}
}


bool compareByPriority(const IcebergData* lhs, const IcebergData* rhs){
	return lhs->priority > rhs->priority;
}


void Drone::updateIcebergData(vector<IcebergData>* icebergData) {
	for (int i = 0; i < currentIcebergData.size() && i < icebergData->size(); ++i) {
		if (icebergData->at(i).timeSinceLastVisit < currentIcebergData[i].timeSinceLastVisit) {
			currentIcebergData[i].priority = icebergData->at(i).priority;
			currentIcebergData[i].timeSinceLastVisit = icebergData->at(i).timeSinceLastVisit;
			currentIcebergData[i].estimatedState = icebergData->at(i).estimatedState;
		}
		if (icebergData->at(i).timeSincePathPlanned < currentIcebergData[i].timeSincePathPlanned
			||(icebergData->at(i).timeSincePathPlanned == currentIcebergData[i].timeSincePathPlanned)
			&& icebergData->at(i).owner != PATH_NO_ICEBERG 
			&& icebergData->at(i).owner < id) {
			currentIcebergData[i].owner = icebergData->at(i).owner;
			currentIcebergData[i].hasBeenBiddenOn= icebergData->at(i).hasBeenBiddenOn;
			currentIcebergData[i].timeSincePathPlanned = icebergData->at(i).timeSincePathPlanned;
			currentIcebergData[i].currentBestPath = icebergData->at(i).currentBestPath;
		}
	}
}



void Drone::updateIcebergData() {
	for (auto it = currentIcebergData.begin(); it != currentIcebergData.end(); ++it) {
		it->timeSinceLastVisit++;
		it->timeSincePathPlanned++;
		it->uppdatePriority(shipState);
	}
}




