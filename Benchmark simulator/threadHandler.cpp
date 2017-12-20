
#define _CRT_SECURE_NO_WARNINGS
#include "threadHandler.h"
#include "stateClasses.h"
#include "Definitions.h"
#include "ship.h"
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <time.h>
#include <iostream>
#include <string>
#include <sstream>
#include <time.h>

using namespace std;


/*
This function calls the different "threads" that are the drones and ship. It also lets time pass for the icebergs
and hadnles the communication between the threads
*/
void threadHandler() {

	int simulationtime = 0;


	/*Setup files*/
	//Saves the drone- and iceberg-states at each timestep
	fstream fileDrones("droneOptimal.txt", fstream::out);
	fstream fileIceberg("icebergOptimal.txt", fstream::out);
	fstream fileInit("init.txt", fstream::in);
	int temp1;
	int temp2;
	//load th init state so that we can have the same initial condition on multiple rounds

	if (!fileDrones.good() || !fileIceberg.good() || !fileInit.good()) {
		cout << "Bad files";
		abort();
	}
	fileInit >> temp1;
	fileInit >> temp2;
	if (temp1 != NUMBER_OF_ICEBERGS || temp2 != NUMBER_OF_DRONES) {
		cout << "Wrong number of icebergs and drones in input file";
		abort();
	}


	
	fileDrones << COMMUNICATION_RADIUS << "\n";


	/*******************/
	/*Initiate board with drones and icebergs*/
	/*******************/
	Ship ship;
	ship.shipState.pos.x = (BOARD_SIZE_X / 2) + BOARD_MIN_X;
	ship.shipState.pos.y = (BOARD_SIZE_Y / 2) + BOARD_MIN_Y;
	ship.shipState.velocity.x = 0;
	ship.shipState.velocity.y = 0;

	Drone drones[NUMBER_OF_DRONES];
	Iceberg icebergs[NUMBER_OF_ICEBERGS];
	vector<IcebergData> icebergData(NUMBER_OF_ICEBERGS);

	//Initialize icebergs, let the data be perfect
	for (int i = 0; i < NUMBER_OF_ICEBERGS; ++i) {
		fileInit >> icebergs[i].state.pos.x;
		fileInit >> icebergs[i].state.pos.y;
				
		icebergs[i].state.velocity.x = 0;
		icebergs[i].state.velocity.y = 0;

		icebergData[i].id = i;
		icebergData[i].estimatedState = icebergs[i].state;
		icebergData[i].timeSinceLastVisit = 1;
		icebergData[i].currentBestPath.reset();
	}
	bool temp;
	for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
		drones[i].id = i;
		drones[i].shipState = ship.shipState;

		fileInit >> temp;
		fileInit >> drones[i].droneState.pos.x;
		fileInit >> drones[i].droneState.pos.y;

		drones[i].currentIcebergData = icebergData;
		drones[i].currentIceberg = 0;
		drones[i].path.reset(drones[i].id);
	}

	ship.currentIcebergData = icebergData;
	/*******************/

	
	/*Load in the path the drones are to follow, indefinitly*/
	fstream fileOptimalPath("optimalPath.txt", fstream::in);
	string line;
	stringstream ss_line;
	double tempDouble;
	double tempInt;
	getline(fileOptimalPath, line); //first line is unusable
	int drone_id;
	int iceberg_id;
	PathNode droneNode;
	while (getline(fileOptimalPath, line)) {
		ss_line << line;
		ss_line >> drone_id;  //Parameters sent to temp variables are ignored
		ss_line >> tempInt;
		ss_line >> tempDouble;
		ss_line >> tempInt;
	
		//reset pathen
		drone_id--; //Convert from 1index to 0index
		drones[drone_id].path.reset(drone_id);
		//set den opp til å ha bare de forskjellige possisjonene, ikke dronen sin egen!
		while (ss_line >> iceberg_id) {
			//ikke ta med isfjell som er nærmere skipet enn korteste avstand
			if (iceberg_id != 0 && distance2D(icebergs[iceberg_id-1].state.pos,ship.shipState.pos) >= MIN_ICEBERG_DISTANCE) {
				iceberg_id--; //Convert from 1index to 0index
				droneNode.iceberg = iceberg_id;
				droneNode.possition = icebergs[iceberg_id].state.pos;
				drones[drone_id].path.nodes.push_back(droneNode);
			}
		}
		ss_line.str("");
		ss_line.clear();
	}





	
	//IncrementTimeCountdown, as the drones will wait for no message to go to the next phase
	//of bidding, we will have to wait and see if that happens. Two itterations without 
	//a message means nothing is happening and we can go to next timestep.
	int incrementTimeCountdown = INCREMENT_TIME_COUNTDOWN_MAX;
	bool incrementTime = true;
	int timePauseTimer = 0;
	bool queueEmpty;

	synchronizeData(drones, &ship); //syncs data between all the drones and the ship


	bool errorSent = false;


	clock_t t1, t2;
	t1 = clock();
	while (simulationtime < MAX_TIME) {
			
		//update the icebergs
		if (incrementTime) {
			ship.run();
			for (int i = 0; i < NUMBER_OF_ICEBERGS; ++i) {
				icebergs[i].run();
			}
		}


		for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
				drones[i].run(incrementTime);
		}


		if (incrementTime) {

			//do communication
			synchronizeData(drones, &ship);
			
			//move the drones 
			for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
				drones[i].droneState.pos += drones[i].droneState.velocity;
			}

			//If a drone is close enough to a iceberg update the data
			for (int d = 0; d < NUMBER_OF_DRONES; ++d) {
				for (int i = 0; i < NUMBER_OF_ICEBERGS; ++i) {
					if (distance2D(drones[d].droneState.pos, icebergs[i].state.pos) <= DRONE_FOW) {
						drones[d].currentIcebergData[i].estimatedState = icebergs[i].state;
						drones[d].currentIcebergData[i].timeSinceLastVisit = 0;
						if (!drones[d].path.finished(drones[d].droneState.pos) && i == drones[d].path.currentNode().iceberg) {
							drones[d].path.incrementCurrentGoal();
						}
					}
				}
			}
		}

		//Save the data
		if (incrementTime) {
			simulationtime++;
			//cout << simulationtime << endl;
			save(icebergs, NUMBER_OF_ICEBERGS, drones, NUMBER_OF_DRONES, ship,fileDrones, fileIceberg);
		}
	}
	t2 = clock();
	fileDrones.close();
	fileIceberg.close();

	float timeDifference = (float)t2 - (float)t1;
	float runTime = timeDifference / CLOCKS_PER_SEC;
	fstream metadata("optimalPathMetadata.txt", fstream::out);
	metadata << "Run time: " << runTime << endl;
	metadata.close();

	system("PAUSE");
}


/*Synchronizes the iceberg data*/
void synchronizeData(Drone* drones, Ship* ship) {
	updateDronesInNetwork(drones, NUMBER_OF_DRONES, ship);
	for (int i = -1; i < NUMBER_OF_DRONES; ++i) {
		for (int j = i + 1; j < NUMBER_OF_DRONES; ++j) {
			if (i != -1) {
				if (find(drones[i].dronesInNetwork.begin(), drones[i].dronesInNetwork.end(), j) != drones[i].dronesInNetwork.end()) {
					//they are close enough to communicate iceberg data, then update it
					drones[i].updateIcebergData(&drones[j].currentIcebergData);
					drones[j].updateIcebergData(&drones[i].currentIcebergData);
				}
			}
			else {
				//i is the ship
				if (find(ship->dronesInNetwork.begin(), ship->dronesInNetwork.end(), j) != ship->dronesInNetwork.end()) {
					//they are close enough to communicate iceberg data, then update it
					ship->updateIcebergData(&drones[j].currentIcebergData);
					drones[j].updateIcebergData(&ship->currentIcebergData);
					drones[j].isUpToDate = true;
				}
			}
		}
	}
}


void save(Iceberg* icebergData, int numberOfIcebergs, Drone* drones, int numberOfDrones, Ship& ship, fstream& droneFile, fstream& icebergFile) {
	//drone
	for (int i = 0; i < numberOfDrones; ++i) {
		droneFile << drones[i].droneState.pos.x << ", " << drones[i].droneState.pos.y << "\n";
		for (auto it = drones[i].path.nodes.begin(); it != drones[i].path.nodes.end(); ++it) {
			droneFile << it->possition.x << ", " << it->possition.y << "\n";
		}
		droneFile << "-1, -1\n";
	}
	droneFile << "-2,-2\n";

	//icebergs
	for (int i = 0; i < numberOfIcebergs; ++i) {
		icebergFile << i << ", " << icebergData[i].state.pos.x << ", " << icebergData[i].state.pos.y << ", " << icebergData[i].state.velocity.x << ", " << icebergData[i].state.velocity.y << ", " <<ship.currentIcebergData[i].priority<<"\n";
	}
	icebergFile << "-2\n";


}


void updateDronesInNetwork(Drone* drones, int numberOfDrones, Ship* ship) {
	//index -1 is the ship
	vector<int> nodes;
	for (int i = -1; i < numberOfDrones; ++i) {
		nodes.push_back(i);
	}


	//Tdod!sorter nodes etter nærhet til skipet mulig det sparer noe tid? 

	vector<int> currentComNetwork;
	vector<int> currentlyAdded;
	int currentNode;
	double distance;
	while (nodes.size()) {
		currentlyAdded.push_back(nodes.front());
		nodes.erase(nodes.begin());

		while(currentlyAdded.size()){
			currentNode = currentlyAdded.front();
			currentComNetwork.push_back(currentNode);
			currentlyAdded.erase(currentlyAdded.begin());

			for (auto it = nodes.begin(); it != nodes.end(); ) {
				if (currentNode == -1) {
					distance = distance2D(ship->shipState.pos, drones[*it].droneState.pos);
				}
				else {
					distance = distance2D(drones[currentNode].droneState.pos, drones[*it].droneState.pos);
				}

				if (distance <= COMMUNICATION_RADIUS) {
					currentlyAdded.push_back(*it);
					it = nodes.erase(it);
				}
				else {
					++it;
				}
			}
		}

		for (auto it = currentComNetwork.begin(); it != currentComNetwork.end(); ++it) {
			if (*it != -1) {
				drones[*it].dronesInNetwork = currentComNetwork;
			}
			else {
				ship->dronesInNetwork = currentComNetwork;
			}
		}
		currentComNetwork.clear();
	}
}
