#include "ship.h"
#include <vector>
using namespace std;

void Ship::updateIcebergData(vector<IcebergData>* icebergData) {
	for (int i = 0; i < currentIcebergData.size() && i < icebergData->size(); ++i) {
		if (icebergData->at(i).timeSinceLastVisit < currentIcebergData[i].timeSinceLastVisit) {
			currentIcebergData[i].priority = icebergData->at(i).priority;
			currentIcebergData[i].timeSinceLastVisit = icebergData->at(i).timeSinceLastVisit;
			currentIcebergData[i].estimatedState = icebergData->at(i).estimatedState;
		}
		if (icebergData->at(i).timeSincePathPlanned < currentIcebergData[i].timeSincePathPlanned) {
			currentIcebergData[i].owner = icebergData->at(i).owner;
			currentIcebergData[i].hasBeenBiddenOn = icebergData->at(i).hasBeenBiddenOn;
			currentIcebergData[i].timeSincePathPlanned = icebergData->at(i).timeSincePathPlanned;
			currentIcebergData[i].currentBestPath = icebergData->at(i).currentBestPath;
		}
	}
}

void Ship::run() {
	updateIcebergData();
}

void Ship::updateIcebergData() {
	for (auto it = currentIcebergData.begin(); it != currentIcebergData.end(); ++it) {
		it->timeSinceLastVisit++;
		it->timeSincePathPlanned++;
		it->uppdatePriority(shipState);
	}
}