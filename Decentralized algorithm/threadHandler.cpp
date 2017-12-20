
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
	fstream fileDrones("drone.txt", fstream::out);
	fstream fileIceberg("iceberg.txt", fstream::out);
	fstream fileInit;
	int temp1;
	int temp2;
	//load th init state so that we can have the same initial condition on multiple rounds
	if (LOAD_INIT_STATE) {
		fileInit.open("init.txt", fstream::in);
		fileInit >> temp1;
		fileInit >> temp2;
		if (temp1 != NUMBER_OF_ICEBERGS || temp2 != NUMBER_OF_DRONES) {
			cout << "Wrong number of icebergs and drones in input file";
			abort();
		}
	}
	else {
		//if we did not want to load, overwrite with the current state
		fileInit.open("init.txt", fstream::out);
	}

	if (!fileDrones.good() || !fileIceberg.good() || !fileInit.good()) {
		cout << "Bad files";
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
		if (!LOAD_INIT_STATE) {
			icebergs[i].state.pos.x = rand() % BOARD_SIZE_X + BOARD_MIN_X;
			icebergs[i].state.pos.y = rand() % BOARD_SIZE_Y + BOARD_MIN_Y;
		}
		else {
			fileInit >> icebergs[i].state.pos.x;
			fileInit >> icebergs[i].state.pos.y;
		}
		
		icebergs[i].state.velocity.x = 0;
		icebergs[i].state.velocity.y = 0;

		icebergData[i].id = i;
		icebergData[i].estimatedState = icebergs[i].state;
		icebergData[i].timeSinceLastVisit = 1;
		icebergData[i].uppdatePriority(ship.shipState);
		icebergData[i].currentBestPath.reset();
	}

	for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
		drones[i].id = i;
		drones[i].shipState = ship.shipState;
		if (!LOAD_INIT_STATE) {
			drones[i].droneState.pos.x = ship.shipState.pos.x + rand() % INITIAL_DRONE_SPREAD - INITIAL_DRONE_SPREAD / 2;
			drones[i].droneState.pos.y = ship.shipState.pos.y + rand() % INITIAL_DRONE_SPREAD - INITIAL_DRONE_SPREAD / 2;
			if (i < NUMBER_LIMITED_DRONES) {
				drones[i].limitedToShipRadius = true;
			}
		}
		else {
			fileInit >> drones[i].limitedToShipRadius;
			fileInit >> drones[i].droneState.pos.x;
			fileInit >> drones[i].droneState.pos.y;
		}
		drones[i].currentIcebergData = icebergData;
		drones[i].currentIceberg = 0;
		drones[i].path.reset(drones[i].id, drones[i].droneState.pos);
	}

	ship.currentIcebergData = icebergData;
	/*******************/


	//If we did not load init state, overwrite it with the data we generated now
	if (!LOAD_INIT_STATE) {
		fileInit << NUMBER_OF_ICEBERGS << endl << NUMBER_OF_DRONES << endl;
		for (int i = 0; i < NUMBER_OF_ICEBERGS; ++i) {
			fileInit << icebergs[i].state.pos.x << ' ' << icebergs[i].state.pos.y << endl;
		}
		for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
			fileInit << drones[i].limitedToShipRadius << ' ' << drones[i].droneState.pos.x << ' ' << drones[i].droneState.pos.y << endl;
		}
		
	}
	fileInit.close();

	//Save everything on the form needed for the optimal solver
	if (!LOAD_INIT_STATE) {
		fstream fileInitOptimal("init_optimal.txt", fstream::out);
		fileInitOptimal << NUMBER_OF_DRONES << ' ' << NUMBER_OF_ICEBERGS << ' ' << NUMBER_OF_DRONES << endl;
		for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
			fileInitOptimal << 0 << ' ' << 10000 << endl;
		}
		for (int i = 0; i < NUMBER_OF_ICEBERGS; ++i) {
			//if (distance2D(icebergs[i].state.pos, ship.shipState.pos) >= MIN_ICEBERG_DISTANCE) {
				fileInitOptimal << i + 1 << ' ' << icebergs[i].state.pos.x << ' ' << icebergs[i].state.pos.y << ' ' << '0' << ' ' << '1' << endl;
				//i+1 since it is one indexed
			//}
		}
		for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
			fileInitOptimal << NUMBER_OF_ICEBERGS +i+ 1 << ' ' << ship.shipState.pos.x << ' ' << ship.shipState.pos.y << ' ' << '0' << endl;
		}
	}

	
	/*
	The message queue is the queue used by the drones to communicate. New messages are pushed to the queue.
	Each cycle the top message is poped out and send to each thread.
	*/
	queue<Message*> messageQueue;
	queue<Message*> trashQueue; //debug queue that shows 

	Message* currentMessage = nullptr;

	//IncrementTimeCountdown, as the drones will wait for no message to go to the next phase
	//of bidding, we will have to wait and see if that happens. Two itterations without 
	//a message means nothing is happening and we can go to next timestep.
	int incrementTimeCountdown = INCREMENT_TIME_COUNTDOWN_MAX;
	bool incrementTime = false;
	int timePauseTimer = 0;
	bool queueEmpty;

	synchronizeData(drones, &ship); //syncs data between all the drones and the ship


	Message *errorMessage;
	bool errorSent = false;
	int numberOfDecisions = 0;
	bool hasBeenReplan = 0;

	clock_t t1, t2;
	t1 = clock();

	while (simulationtime < MAX_TIME) {
	
		
		/*Find if multiple drones have the same iceberg in their path, if so then send an error message*/
		/*if (incrementTime) errorSent = false;
		for (int i = 0; i < NUMBER_OF_DRONES && !errorSent; ++i) {
			for i(auto it = drones[i].path.nodes.begin(); it != drones[i].path.nodes.end(); ++it) {
				if (it->iceberg == PATH_NO_ICEBERG) continue;
				for (int j = 0; j < NUMBER_OF_DRONES && !errorSent; ++j) {
					if (i == j) continue;
					if (find(drones[j].path.nodes.begin(), drones[j].path.nodes.end(),*it) != drones[j].path.nodes.end()) {
#ifdef DOPRINT
						cout << "Multiplle drones on an iceberg!";
#endif // DOPRINT

						if (find(drones[i].dronesInNetwork.begin(), drones[i].dronesInNetwork.end(), j) != drones[i].dronesInNetwork.end()) {
							//If they are in com, send an error message 
							errorMessage = new Message();
							errorMessage->type = msg_error;
							errorMessage->sendPos = drones[i].droneState.pos;
							errorMessage->senderId = i;
							messageQueue.push(errorMessage);
							errorMessage = nullptr;
							errorSent = true;
						}

					}
				}
			}
		}*/

		

		//Let time pass if all drones are in the flying state
		incrementTime = true;
		for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
			if (drones[i].currentState != state_flying) {
				incrementTime = false;
				//++timePauseTimer;
				break;
			}
		}
		//if (incrementTime) timePauseTimer = 0;
		/*if (timePauseTimer > 1000) {
			cout << "Seems like time is not going to start again. What an inconvenience.";
		}*/


		//Get the current message
		if (!messageQueue.empty()) {
			currentMessage = messageQueue.front();

			if (!hasBeenReplan && currentMessage->type == msg_reMakePath) {
				hasBeenReplan = true;
				numberOfDecisions++;
			}

		}
		else {
			currentMessage = nullptr;
		}
		queueEmpty = messageQueue.empty();



		
		//update the icebergs
		if (incrementTime) {
			hasBeenReplan = false;
			ship.run();
			for (int i = 0; i < NUMBER_OF_ICEBERGS; ++i) {
				icebergs[i].run();
			}
		}


		//If they are remaking the path, then make sure that they have the same information
		//before they start
		if (currentMessage != nullptr && currentMessage->type == msg_reMakePath) {
			synchronizeData(drones, &ship);
		}


		for (int i = 0; i < NUMBER_OF_DRONES; ++i) {
			//do not send messages to drones that are to far away
			//dont send the message to the sender
			if (currentMessage != nullptr 
				&& currentMessage->senderId != i 
				&& find(drones[i].dronesInNetwork.begin(), drones[i].dronesInNetwork.end(), currentMessage->senderId) != drones[i].dronesInNetwork.end()) {
				drones[i].run(currentMessage, &messageQueue, queueEmpty, incrementTime);
			}
			else {
				drones[i].run(nullptr, &messageQueue, queueEmpty, incrementTime);
			}
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
						drones[d].currentIcebergData[i].uppdatePriority(drones[d].shipState);
						if (!drones[d].path.finished(drones[d].droneState.pos) && i == drones[d].path.currentNode().iceberg) {
							drones[d].path.incrementCurrentGoal();
						}
					}
				}
			}
		}


		//Delete the message (push it to the trash queue if we are debuging
#ifdef DEBUG
		if (currentMessage) {
			messageQueue.pop();
		}
		trashQueue.push(currentMessage);
		if (trashQueue.size() > 40) {
			if (trashQueue.front()) {
				delete trashQueue.front();
			}
			trashQueue.pop();
		}
#else
		if (currentMessage != nullptr) {
			delete currentMessage;
			messageQueue.pop();
		}
#endif // DEBUG

		currentMessage = nullptr;


		//Save the data
		if (incrementTime) {
			simulationtime++;
#ifdef DOPRINT
			cout << simulationtime << endl;
#endif // DOPRINT
			save(icebergs, NUMBER_OF_ICEBERGS, drones, NUMBER_OF_DRONES, ship,fileDrones, fileIceberg);
		}
	}
	t2 = clock();
	fileDrones.close();
	fileIceberg.close();
	float timeDiff = (float)t2 - (float)t1;
	float runTime = timeDiff / CLOCKS_PER_SEC;
	cout << "Number of routes planned: " << numberOfDecisions << "\nRunning time: " << runTime << endl;
	fstream metadata("metadata.txt", fstream::out);
	metadata << "Running time: " << runTime << endl;
	metadata << "Number of routes planned: " << numberOfDecisions << endl;
	metadata << "Number of drones: " << NUMBER_OF_DRONES << endl;
	metadata << "Number of icebergs: " << NUMBER_OF_ICEBERGS << endl;
	metadata << "Number of limited drones: " << NUMBER_LIMITED_DRONES << endl;
	metadata << "Shortest allowed path: " << DRONE_SHORTEST_PATH << endl;
	metadata << "Priority scaler: " << DRONE_PRIORITY_SCALER << endl;
	metadata << "Movement speed: " << DRONE_MOVEMENT_SPEED << endl;
	metadata << "Field of view: " << DRONE_FOW << endl;
	metadata << "Com node slack: " << COM_NODE_SLACK << endl;
	metadata << "Tolate penalty: " << TOLATE_PENALTY << endl;
	metadata << "Max number of itterations for com node path planning: " << MAX_NUMBER_ITERATIONS << endl;
	metadata << "Use optimal return path: " << USE_OPTIMAL_RETURN_PATH << endl;
	metadata << "Max iceberg distance: " << MAX_ICEBERG_DISTANCE << endl;
	metadata << "Min iceberg distance: " << MIN_ICEBERG_DISTANCE << endl;
	metadata << "Icebergdata distance scaler: " << ICEBERGDATA_DISTANCE_SCALER << endl;
	metadata << "Communication radius: " << COMMUNICATION_RADIUS << endl;
	metadata << "Minimum com assistance distance: " << MINIMUM_COM_ASSIST_DISTANCE << endl;
	metadata << "Planned communication radius: " << PLANNED_COMMUNICATION_RADIUS << endl;
	metadata << "Initial drone spread: " << INITIAL_DRONE_SPREAD << endl;
	metadata << "Board size x: " << BOARD_SIZE_X << endl;
	metadata << "Board size y: " << BOARD_SIZE_Y << endl;
	metadata << "Min board x value: " << BOARD_MIN_X << endl;
	metadata << "Min board y value " << BOARD_MIN_Y << endl;
	metadata << "Longest waiting time at com-node: " << TIMEOUTMAX << endl;
	metadata << "Number of timesteps: " << MAX_TIME;
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
		for (auto it = drones[i].path.nodes.begin() + 1; it != drones[i].path.nodes.end(); ++it) {
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
