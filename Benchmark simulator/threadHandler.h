#pragma once
#include "Iceberg.h"
#include "Drone.h"
#include "ship.h"

void threadHandler();
void save(Iceberg* icebergData, int numberOfIcebergs, Drone* drones, int numberOfDrones, Ship& ship, std::fstream &droneFile, std::fstream &icebergFile);
void updateDronesInNetwork(Drone* drones, int numberOfDrones, Ship* shipPos);
void synchronizeData(Drone* drones, Ship* ship);