#include "Drone.h"
#include "BellmanFord.h"
#include <iostream>
#include <algorithm>
using namespace std;

Drone::Drone() {
	path.reset();
}


void Drone::run(const Message* inputMessage, std::queue<Message*>* messageQue, bool messageQueueEmpty, bool incrementTime) {

	//Increment time må skje først siden den resetter stuff som kan bli satt når incrementTime sendes inn
	if (incrementTime) {
		hasRecievedReMakePath = false;
		hasStartedReMakePath = false;

		updateIcebergData();

		if (!isInComWithShip()) {
			isUpToDate = false;
		}
	}
	Message* outputMessage;

	//update if we are close to a com node, this is not handled by the thread handler as it only handles in
	//relation to real icebergs
	if (!path.finished(droneState.pos) && path.currentNode().iceberg == PATH_NO_ICEBERG
		&& distance2D(droneState.pos,path.currentNode().possition)<COM_NODE_SLACK){
		path.incrementCurrentGoal();
	}


	/*Handle communication*/
	if (inputMessage) {
		switch (inputMessage->type) {
		case msg_reMakePath:
			if (hasRecievedReMakePath) break;
			currentState = state_proposingNodes;
			hasRecievedReMakePath = true;
			break;
		case msg_proposeNode:
			proposedNodes.push_back(inputMessage->nodeProposal);
			break;
		case msg_pathProposal:
			/*if (currentState == state_flying) {
#ifdef DOPRINT
				cout << "Er i feil state!";
#endif // DOPRINT
			
				outputMessage = new Message();
				outputMessage->type = msg_error;
				outputMessage->senderId = id;
				outputMessage->sendPos = droneState.pos;
				messageQue->push(outputMessage);
				path.reset(id, droneState.pos);
			}*/

			//if the message received is better, or equally good and the sender has a lower id. (Lower ID gets priority if the paths are equally good)
			if (inputMessage->path.objectiveValue < currentIcebergData[inputMessage->path.endIcebergId()].currentBestPath.objectiveValue
				|| inputMessage->path.objectiveValue == currentIcebergData[inputMessage->path.endIcebergId()].currentBestPath.objectiveValue
				&& inputMessage->senderId < currentIcebergData[inputMessage->path.endIcebergId()].currentBestPath.owner) {
				currentIcebergData[inputMessage->path.endIcebergId()].currentBestPath = inputMessage->path;
			}
			break;
		case msg_error:
			currentState = state_flying;
			break;
		case msg_newNode:
			currentIcebergData.push_back(inputMessage->newNode);
			sortIcebergData();
#ifdef DOPRINT
			cout << "received new node\n";
#endif // DOPRINT
			break;
		}
	}
	


	/*Handle state*/
	Path tempPath;
	tempPath.reset();

	outputMessage = new Message();
	outputMessage->type = msg_noMessage;
	outputMessage->senderId = id;
	outputMessage->sendPos = droneState.pos;

	switch (currentState){
	case state_flying:
		if (incrementTime) {
			Vector2D currentPosGoal = shipState.pos; //default location

			/*if (path.currentGoal < path.nodes.size()) {
				if (path.currentNode().isComNode) {

					cout << "this";
				}
			}*/

			if (!path.finished(droneState.pos)) {
				currentPosGoal = path.currentNode().possition;
			}
			else {
				if (path.previousNode().isComNode && path.previousNode().getsCom) {
					cout << "dette skal ikke være tilfelle...";
				}


				if (path.previousNode().isComNode
					&& (!timerStartet || timeoutTimer>0) ) {
					//we are finished due to the drone being at a node which is a com node
					//Then stay put untill communicaition (and replanning), or timeout
					currentPosGoal = droneState.pos;
					if (!timerStartet) {
						timeoutTimer = TIMEOUTMAX;
						timerStartet = true;
					}
					else {
						--timeoutTimer;
					}
				}
				else if (!hasRecievedReMakePath && isInComWithShip() && isUpToDate) {
					//We are in com with the ship, so reMake!
					currentState = state_proposingNodes;
					hasRecievedReMakePath = true;
					outputMessage->type = msg_reMakePath;
					//currentPosGoal = droneState.pos;
				}
				//if we are not in com with ship, fly towards the ship untill we are
			}

			Vector2D vectorToGoal = currentPosGoal - droneState.pos;
			normalize(&vectorToGoal);
			droneState.velocity = vectorToGoal* DRONE_MOVEMENT_SPEED;
		}
		
		break;
	case state_proposingNodes:
		if (!hasStartedReMakePath) {
			//If we are waiting on a com node that is owned by a drone not in communication, then dont join the replanning
			//continue with the last path that ended in that com-node
			/*for (auto n = path.nodes.begin(); n != path.nodes.end(); ++n) {
				if (n->isComNode) {
					if (find(dronesInNetwork.begin(), dronesInNetwork.end(), currentIcebergData[n->iceberg].comNodeOwner) != dronesInNetwork.end()) {
						currentState = state_flying;
						//Nå kan de bli doppelt opp på de rutene som var i pathen til denne dronen siden de andre sletter dem...
						break;
					}
				}
			}*/

			//Should only happen the first time we are in this state
			setupRemakePath();
		}

		if (!hasSentNode && messageQueueEmpty) {
			proposedNodes.clear();
			hasSentNode = true;
			if (path.size() - 1 <= DRONE_SHORTEST_PATH) {
				//we still need to get more nodes in our path untill we are satisfied
				//find a node and bid on that
				int nextNode = findNextFreeIceberg();
				if (nextNode != DRONE_NO_MORE_ICEBERGS) {
					outputMessage->type = msg_proposeNode;
					outputMessage->nodeProposal.nodeId = nextNode;
					outputMessage->nodeProposal.priority = currentIcebergData[nextNode].priority;
					proposedNodes.push_back(outputMessage->nodeProposal);
				}
			}
			//we are satisfied so no need proposing a node
		}
		else if (messageQueueEmpty) {
			//everyone has sent their proposals
			if (proposedNodes.size() == 0) {
				//Fix the path to be the best route home
				if (USE_OPTIMAL_RETURN_PATH
					&& distance2D(path.nodes.back().possition, shipState.pos) > COMMUNICATION_RADIUS) {
					if (!path.nodes.back().getsCom) {
						//If we do not get assistance
						//delete nodes that are not icebergs and not com-nodes
						for (auto it = path.nodes.begin()+1; it != path.nodes.end();) {
							if (!it->isComNode && it->iceberg == PATH_NO_ICEBERG) {
								it = path.nodes.erase(it);
							}
							else {
								++it;
							}
						}
						int baseLength = path.size();
						//Make the best path to the ship possition
						Path pathWithReturn;
						IcebergData shipPos;
						shipPos.id = currentIcebergData.size();
						shipPos.estimatedState.pos = shipState.pos;
						currentIcebergData.push_back(shipPos);
						calculateBestPath(currentIcebergData.size()-1, &path, &pathWithReturn, DRONE_PRIORITY_SCALER);
						path = pathWithReturn;
						//delete all nodes get com-at the end
						for (auto it = path.nodes.begin() + baseLength; it != path.nodes.end(); ++it) {
							if (distance2D(currentIcebergData[it->iceberg].estimatedState.pos, shipState.pos) <= COMMUNICATION_RADIUS) {
								path.nodes.erase(it, path.nodes.end());
								break;
							}
						}
						currentIcebergData.pop_back();

					}					
				}


				//all drones are satisfied, so go to flying
				currentState = state_flying;
				currentIceberg = 0;
			}
			else {
				//find the best one and set that as currentIceberg
				//using reverse itterators to sort in decending order.
				sort(proposedNodes.rbegin(), proposedNodes.rend()); //the sorted list should be equal for all threads
				currentIceberg = proposedNodes[0].nodeId;
				currentState = state_bidding;
			}
			//proposedNodes.clear();
			hasSentNode = false;
		}


		break;
	case state_bidding:
		if (currentIceberg >= currentIcebergData.size()) {
			currentState = state_waitingForBids;
			cout << "Har ikke det isfjellet???";
			break;
		}
		if (!limitedToShipRadius || distance2D(shipState.pos, currentIcebergData[currentIceberg].estimatedState.pos) < PLANNED_COMMUNICATION_RADIUS) {
			//If the ship is limited to ship radius, then the iceberg we are bidding on should be close


			if (currentIcebergData[currentIceberg].isComNode) {
				if (currentIcebergData[currentIceberg].comNodeOwner == id
					|| distance2D(currentIcebergData[currentIceberg].estimatedState.pos, droneState.pos) > currentIcebergData[currentIceberg].inactivationTime + PLANNED_COMMUNICATION_RADIUS
					|| path.allreadyHasAComNode) {
					//hvis vi allerede har en com node ønsker vi ikke å by på en til. Enten det er at vi får, eller gir kom.
					//Nå er det ikke sikkert den klarer å nå i tide, sorterer bare ut de som ikke klarer det
					currentState = state_waitingForBids;
					currentIcebergData[currentIceberg].hasBeenBiddenOn = true;
					break;
				}

				bool didFindPath = calculateBestPathToComNode(currentIceberg, &path, &tempPath, currentIcebergData[currentIceberg].activeTime, currentIcebergData[currentIceberg].dronePos, currentIcebergData[currentIceberg].shipPos);
				if (!didFindPath) {
					currentState = state_waitingForBids;
					currentIcebergData[currentIceberg].hasBeenBiddenOn = true;
#ifdef DOPRINT
					cout << "failed to get to com node\n";
#endif // DOPRINT				
					break;
				}
#ifdef DOPRINT
				cout << "found a com path\n";
#endif // DOPRINT
			}
			else {
				calculateBestPath(currentIceberg, &path, &tempPath, DRONE_PRIORITY_SCALER);
			}

			if (tempPath.objectiveValue < currentIcebergData[currentIceberg].currentBestPath.objectiveValue
				|| tempPath.objectiveValue == currentIcebergData[currentIceberg].currentBestPath.objectiveValue
				&& id < currentIcebergData[currentIceberg].currentBestPath.owner) {

				currentIcebergData[currentIceberg].currentBestPath = tempPath;
				outputMessage->type = msg_pathProposal;
				outputMessage->path = tempPath;
			}
		}

		currentState = state_waitingForBids;
		break;
	case state_waitingForBids:
		if (messageQueueEmpty) {
			//everyone has answered
			//Set the ownership for the nodes in the winning path
			setOwnershipFromPath(&(currentIcebergData[currentIceberg].currentBestPath));

			//If we had the winning path, change our current path to that one
			//hvis cuurentIceberg er en kommunikasjonsnode, og vi ikke tok den, slett den.
			if(currentIcebergData[currentIceberg].owner == id) {
				path = currentIcebergData[currentIceberg].currentBestPath;

				if (currentIcebergData[currentIceberg].isComNode) {
					path.allreadyHasAComNode = true;
				}

				if (!path.allreadyHasAComNode) {
					//Om jeg allerede ikke har en node som er markert som trenger kom
					//om jeg trenger kom i endepunktet
					double distance = distance2D(path.nodes.back().possition, shipState.pos);
					if (distance > MINIMUM_COM_ASSIST_DISTANCE) {
						if (distance > 2 * PLANNED_COMMUNICATION_RADIUS) {
							//Om avstanden min til skipet er > 2 komminukasjons radie, 
							//legg til en node til som siste node, plasser den langs den rette linja fra siste node til skipet,
							//akkuratt på 2* kommunikasjonsradien
							PathNode temp;
							temp.possition = (normalize(path.nodes.back().possition-shipState.pos) *2 * PLANNED_COMMUNICATION_RADIUS) + shipState.pos;
							temp.priority = path.nodes.back().priority;
							temp.iceberg = PATH_NO_ICEBERG;
							path.nodes.push_back(temp);
						}

						path.nodes.back().needsCom = true;
						path.allreadyHasAComNode = true;
						outputMessage->type = msg_newNode;
						outputMessage->newNode.id = currentIcebergData.size();
						outputMessage->newNode.priority = path.nodes.back().priority;
						outputMessage->newNode.estimatedState.pos = shipState.pos + (path.nodes.back().possition - shipState.pos) * 0.5;
						outputMessage->newNode.activeTime = lengthOfPath(&path);
						//the communication constrained is deactivated when the drone has returned to com without com-help
						//time to the com sircle of the ship, is distance to ship minus com radius added to the length of the path
						outputMessage->newNode.inactivationTime = outputMessage->newNode.activeTime + distance2D(path.nodes.back().possition, shipState.pos) - PLANNED_COMMUNICATION_RADIUS;
						outputMessage->newNode.isComNode = true;
						outputMessage->newNode.comNodeOwner = id;
						outputMessage->newNode.dronePos = path.nodes.back().possition;
						outputMessage->newNode.shipPos = shipState.pos;
						outputMessage->newNode.currentBestPath.reset();
						currentIcebergData.push_back(outputMessage->newNode);
						sortIcebergData();
#ifdef DOPRINT
						cout << "sent new node \n";
#endif // DOPRINT						
					}
				}
			}

			if (currentIcebergData[currentIceberg].isComNode && currentIcebergData[currentIceberg].owner != NO_OWNER
				&& currentIcebergData[currentIceberg].comNodeOwner == id) {
				path.nodes.back().getsCom = true;
			}

			currentState = state_proposingNodes;
		}
		break;
	default:
		break;
	}

	if (outputMessage->type == msg_noMessage) {
		delete outputMessage;
	}
	else {
		messageQue->push(outputMessage);
	}
	outputMessage = nullptr;
}

void Drone::setupRemakePath() {
	//remove all commnication nodes
	currentIcebergData.erase(currentIcebergData.begin() + NUMBER_OF_ICEBERGS, currentIcebergData.end());

	for (auto it_drones = dronesInNetwork.begin(); it_drones != dronesInNetwork.end(); ++it_drones) {
		removeOwnershipByDrone(*it_drones);
	}
	sortIcebergData();
	currentIceberg = findNextFreeIceberg();
	path.reset(id, droneState.pos);
	hasStartedReMakePath = true;
	timerStartet = false;

	proposedNodes.clear();
	
}

bool compareByPriority(const IcebergData* lhs, const IcebergData* rhs){
	return lhs->priority > rhs->priority;
}

void Drone::sortIcebergData() {
	//todo dette var da lite optimalt
	if (currentIcebergData_byPriority.size() != currentIcebergData.size()) {
		currentIcebergData_byPriority.clear();
		for (auto it = currentIcebergData.begin(); it != currentIcebergData.end(); ++it) {
			currentIcebergData_byPriority.push_back(&(*it));
		}
	}
	sort(currentIcebergData_byPriority.begin(), currentIcebergData_byPriority.end(), compareByPriority);
}

int Drone::findNextFreeIceberg() const {
	for (int i = 0; i < currentIcebergData_byPriority.size(); ++i) {
		if (!currentIcebergData_byPriority[i]->hasBeenBiddenOn) {
			if (!limitedToShipRadius 
				|| distance2D(shipState.pos, currentIcebergData_byPriority[i]->estimatedState.pos) < PLANNED_COMMUNICATION_RADIUS) {
				return currentIcebergData_byPriority[i]->id;
			}
		}
	}
	return DRONE_NO_MORE_ICEBERGS;
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

void Drone::removeOwnershipByDrone(int droneId) {
	for (auto it = currentIcebergData.begin(); it != currentIcebergData.end(); ++it) {
		if (it->owner == droneId) {
			it->owner = ICEBERGDATA_NO_OWNER;
			it->hasBeenBiddenOn = false;
			it->currentBestPath.reset();
			it->timeSincePathPlanned = 0;
		}
	}
}

void Drone::setOwnershipFromPath(Path* path) {
	for (auto it = path->nodes.begin(); it != path->nodes.end(); ++it) {
		if (it->iceberg != PATH_NO_ICEBERG) {
#ifdef DOPRINT
			if (currentIcebergData[it->iceberg].owner != ICEBERGDATA_NO_OWNER 
				&& currentIcebergData[it->iceberg].owner != path->owner) {
				cout << "Overskriver eier!";
			}
#endif // DOPRINT
			currentIcebergData[it->iceberg].owner = path->owner;
			currentIcebergData[it->iceberg].hasBeenBiddenOn = true;;
			currentIcebergData[it->iceberg].timeSincePathPlanned = 0;
		}
	}
}

bool Drone::calculateBestPathToComNode(int goalIcebergId, const Path* basePath, Path* returnPath, double timepoint, Vector2D dronePos, Vector2D shipPos) {

	bool returnVal = false;
	//todo test om gain skal endre seg etter hvor mye vi var for tidlig/sene
	double gain = 4;
	double gainChange = 0.8;
	double priorityScaler = DRONE_PRIORITY_SCALER;
	double pathLength;
	Vector2D possition;

	int highestPrio = currentIcebergData_byPriority[0]->priority;

	Path tempPath;
	double bestSumPriority = 0;
	double sumPriority;

	int lastNode = 0;

	for (int i = 0; i < MAX_NUMBER_ITERATIONS; ++i) {
		calculateBestPath(goalIcebergId, basePath, &tempPath, priorityScaler);
		//Remake the objective value based on the real priority_scaler
		/*tempPath.objectiveValue = 0;
		sumPriority = 0;
		auto jt = tempPath.nodes.begin() + 1;
		for (auto it = tempPath.nodes.begin(); jt != tempPath.nodes.end(); ++it) {
			tempPath.objectiveValue += distance2D(jt->possition, it->possition);
			tempPath.objectiveValue -= jt->priority * DRONE_PRIORITY_SCALER / highestPrio;
			sumPriority += jt->priority;
			++jt;
		}*/

		pathLength = lengthOfPath(&tempPath);
		if (pathLength < timepoint) {

			tempPath.objectiveValue = 0;
			sumPriority = 0;
			auto jt = tempPath.nodes.begin() + 1;
			for (auto it = tempPath.nodes.begin(); jt != tempPath.nodes.end(); ++it) {
				tempPath.objectiveValue += distance2D(jt->possition, it->possition);
				tempPath.objectiveValue -= jt->priority * DRONE_PRIORITY_SCALER / highestPrio;
				sumPriority += jt->priority;
				++jt;
			}

			//we arrive to early
			//But it is a valid path, so we save it, but continue in the hopes of finding a better one
			//if (tempPath.objectiveValue < returnPath->objectiveValue) {

			if (sumPriority > bestSumPriority) {
				bestSumPriority = sumPriority;
				*returnPath = tempPath;
				returnVal = true;
			}
			
			//increase priority scaler
			priorityScaler *= (1+gain);
			
		}
		else {
			//find our location at timepoint
			
			possition = locationAtTimepoin(&tempPath, timepoint, &lastNode);
			if (distance2D(possition, dronePos) <= PLANNED_COMMUNICATION_RADIUS && distance2D(possition, shipPos) <= PLANNED_COMMUNICATION_RADIUS) {

				//slett alle elementene fra og med lastNode til og uten siste element som er kom-noden
				tempPath.nodes.erase(tempPath.nodes.begin() + lastNode, tempPath.nodes.end() - 1);
				//recalc priority
				tempPath.objectiveValue = 0;
				sumPriority = 0;
				auto jt = tempPath.nodes.begin() + 1;
				for (auto it = tempPath.nodes.begin(); jt != tempPath.nodes.end(); ++it) {
					tempPath.objectiveValue += distance2D(jt->possition, it->possition);
					tempPath.objectiveValue -= jt->priority * DRONE_PRIORITY_SCALER / highestPrio;
					sumPriority += jt->priority;
					++jt;
				}


				//we are at a good point
				//if (tempPath.objectiveValue < returnPath->objectiveValue) {
				if (sumPriority > bestSumPriority) {
					bestSumPriority = sumPriority;
					*returnPath = tempPath;
					returnVal = true;
				}
				return true;
			}
			else {
				//we are to late
				//reduce priority scaler
				priorityScaler *= 1/(1+gain);


				//todo if the drone is able to get back into com before we are able to get there, then ignore the path
				

				//if we are flying in a direct line
				if (tempPath.nodes.size() == basePath->nodes.size() + 1) {
					tempPath.objectiveValue = 0;
					auto jt = tempPath.nodes.begin() + 1;
					for (auto it = tempPath.nodes.begin(); jt != tempPath.nodes.end(); ++it) {
						tempPath.objectiveValue += distance2D(jt->possition, it->possition);
						tempPath.objectiveValue -= jt->priority * DRONE_PRIORITY_SCALER / highestPrio;
						++jt;
					}

					//we are not going to be able to get there in time
					tempPath.objectiveValue += TOLATE_PENALTY*(pathLength - timepoint);
					*returnPath = tempPath;
					return false;
				}
			}
		}
		gain *= gainChange;
	}

	return returnVal;
}

bool Drone::calculateBestPath(int goalIcebergId, const Path* basePath, Path* returnPath, double priorityScaler) {
	//We should find the path from the last node in the base path
	
	int source = basePath->endIcebergId();
	if (source == PATH_NO_ICEBERG) source = currentIcebergData.size();

	int highestPrio = currentIcebergData_byPriority[0]->priority;

	//In the Bellman Ford algorithm, let our possition be indexed as node currentIcebergData.size()

	vector<Edge> edges;
	double weight;
	double dist;
	//add a path from our possition to every node, if we start at our possition
	//if we start at a node, just do from every node to another.
	if (basePath->nodes.back().iceberg == PATH_NO_ICEBERG) {
		Vector2D startPossition = basePath->nodes.back().possition;
		for (int i = 0; i < currentIcebergData.size(); ++i) {
			if (i != goalIcebergId && currentIcebergData[i].owner != ICEBERGDATA_NO_OWNER) continue;
			dist = distance2D(currentIcebergData[i].estimatedState.pos, startPossition);
			weight = dist - currentIcebergData[i].priority * priorityScaler  / highestPrio;
			
			edges.push_back(Edge(source, i, weight));
		}
	}
	

	//From a node to the next
	double iDist;
	double jDist;
	double ijDist;

	//we want edges from com nodes to others, as the com node may be part of the path
	//but not from others to com unless the com node is the end node
	

	for (int i = 0; i < currentIcebergData.size(); ++i) {
		if (i != source && i != goalIcebergId && currentIcebergData[i].owner != ICEBERGDATA_NO_OWNER) continue;
		for (int j = i+1; j < currentIcebergData.size(); ++j) {
			if (j != source && j != goalIcebergId && currentIcebergData[j].owner != ICEBERGDATA_NO_OWNER) continue;
			//ignore edges that already has a owner, ignore com nodes as we dont want them as a part of our path, unless its the goal
			iDist = distance2D(currentIcebergData[goalIcebergId].estimatedState.pos, currentIcebergData[i].estimatedState.pos);
			jDist = distance2D(currentIcebergData[goalIcebergId].estimatedState.pos, currentIcebergData[j].estimatedState.pos);

			ijDist = distance2D(currentIcebergData[i].estimatedState.pos, currentIcebergData[j].estimatedState.pos);
			if ((i == source && !currentIcebergData[j].isComNode) || j == goalIcebergId) {
				weight = ijDist - currentIcebergData[j].priority * priorityScaler / highestPrio;
				edges.push_back(Edge(i, j, weight));
			}
			else if ((j == source && !currentIcebergData[i].isComNode) || i == goalIcebergId) {
				weight = ijDist - currentIcebergData[i].priority * priorityScaler / highestPrio;
				edges.push_back(Edge(j, i, weight));
			}
			else if (jDist > iDist && !currentIcebergData[i].isComNode) {
				weight = ijDist - currentIcebergData[i].priority * priorityScaler / highestPrio;
				edges.push_back(Edge(j, i, weight));
			}
			else if(iDist > jDist && !currentIcebergData[j].isComNode ){
				weight = ijDist - currentIcebergData[j].priority * priorityScaler / highestPrio;
				edges.push_back(Edge(i, j, weight));
			}
			//if they are equal, dont push the edge as this can produce cycles 
		}
	}

	vector<int> bestPath;
	double pathLength;

	bool noNegativeCycles = bellmanFord(edges, currentIcebergData.size()+1, source,goalIcebergId,&bestPath,&pathLength);

	if (!noNegativeCycles) {
		cout << "Negative cycle!";
		system("PAUSE");
		abort();
		return false;
	}

	*returnPath = *basePath;
	
	returnPath->objectiveValue += pathLength;
	PathNode node;
	//drops the first, as that is the start node which is already a part of the path
	for (auto it = bestPath.begin()+1; it != bestPath.end(); ++it) {
		node.possition = currentIcebergData[*it].estimatedState.pos;
		node.iceberg = *it;
		node.priority = currentIcebergData[*it].priority;
		node.isComNode = currentIcebergData[*it].isComNode;
		returnPath->nodes.push_back(node);
	}

	if (returnPath->objectiveValue == 0) returnPath->objectiveValue = INFINITY;

	return true;
}


void Drone::updateIcebergData() {
	for (auto it = currentIcebergData.begin(); it != currentIcebergData.end(); ++it) {
		it->timeSinceLastVisit++;
		it->uppdatePriority(shipState);
		it->timeSincePathPlanned++;
	}
}








/*not in use*/
/*
bool Drone::testBiddingFinished() {
	int numberOfHandledIcebergs = 0;
	for (auto it = currentIcebergData_byPriority.begin(); it != currentIcebergData_byPriority.end(); ++it) {
		if (!(*it)->hasBeenBiddenOn) break;
		numberOfHandledIcebergs++;
	}
	if (numberOfHandledIcebergs < DRONE_MIN_NUM_HANDLED_ICEBERGS) return false;

	vector<int> dronesInNetwork_copy = dronesInNetwork;
	//remove the ship
	auto shipIt = find(dronesInNetwork_copy.begin(), dronesInNetwork_copy.end(), -1);
	if (shipIt != dronesInNetwork_copy.end()) {
		dronesInNetwork_copy.erase(shipIt);
	}


	for (auto it = currentIcebergData_byPriority.begin(); it != currentIcebergData_byPriority.end(); ++it) {
		auto droneIt = find(dronesInNetwork_copy.begin(), dronesInNetwork_copy.end(), (*it)->owner);
		if (droneIt != dronesInNetwork_copy.end()) {
			dronesInNetwork_copy.erase(droneIt);
		}
		if (dronesInNetwork_copy.empty()) return true;
	}
	if (dronesInNetwork_copy.empty()) return true;
	else return false;
}

*/
