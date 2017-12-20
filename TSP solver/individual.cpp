#include "individual.h"
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include "Parameters.h"
#include "alg.h"
#include <iostream>

using namespace std;


individual::individual(problemData* data) {
	this->data = data;

	depos.assign(data->num_Depots, vector<int>(0, 0));

	// Assign kunder til nærmeste depo
	//Gå gjennom alle kundene

	
	//lag numDepos random plaserte punkter, asign til nermeste
	vector<pos> routeCentres;
	pos possition;
	for (int i = 0; i < data->num_Depots; ++i) {
		possition.x = rand() % BOARD_SIZE;
		possition.y = rand() % BOARD_SIZE;
		routeCentres.push_back(possition);
	}



	double bestDistance;
	int bestDepo;
	double currentDistance;
	for (auto c = data->customers.begin()+1; c != data->customers.end(); ++c) {
		//Finn nermeste depo
		bestDistance = INFINITY;
		bestDepo = -1;
		for (int d = 0; d < data->num_Depots; ++d) {
			currentDistance = custommerPosDistance(*c, routeCentres[d]);
			if (currentDistance < bestDistance) {
				bestDistance = currentDistance;
				bestDepo = d;
			}
		}
		depos[bestDepo].push_back(c->id);
	}

	// Shuffle kunderekkefølgen tilfeldig 
	//(gjør at de forskjellige kromosonene i initialbefolkningen er forskjellige)
	for (auto d = depos.begin(); d != depos.end(); ++d) {
		random_shuffle(d->begin(), d->end());
	}
}

//returnes length, not fitness
double individual::calc_fitness() {

	fitness = 0;
	length = 0;
	isInfeasable = 0;

	double max_load;
	double max_duration;
	double max_num_vehicle_per_route = data->num_Vehicles;

	int curNumVehicle = 0;

	double prevX = 0;
	double prevY = 0;
	double currX = 0;
	double currY = 0;

	double depotX = 0;
	double depotY = 0;

	vector<vector<int>> routes;

	depoRoutes.clear();

	double currentLength;
	double sumLength = 0;
	double maxLength =0;
	bool isFirst;
	double returnDistance;
	double entryDistance;
	for (int d = 0; d < data->num_Depots; ++d) {
		//max_load = data->depots[d].max_load;
		//max_duration = data->depots[d].max_duration;
		currentLength = 0;
		depotX = data->depots[d].x;
		depotY = data->depots[d].y;

		curNumVehicle = 0;
		//bool success = makeRoutes(depos[d], d, &routes, data
		/*if (!success) {
			FAILED = 1;
			fitness = INFINITY;
			length = fitness;
		}*/
		routes.clear();
		routes.push_back(depos[d]);
		depoRoutes.push_back(routes);
		isFirst = true;
		for (auto r = routes.begin(); r != routes.end(); ++r) {
			prevX = depotX;
			prevY = depotY;
			for (auto c = r->begin(); c != r->end(); ++c) {
				currX = data->customers[*c].x;
				currY = data->customers[*c].y;

				if (isFirst) {
					//den første er avstanden fra depotet til første node
					isFirst = false;
					entryDistance = sqrt(pow(currX - prevX, 2) + pow(currY - prevY, 2));
					if (entryDistance > COMMUNICATION_RADIUS) {
						currentLength += entryDistance - COMMUNICATION_RADIUS;
					}
				}
				else {
					currentLength += sqrt(pow(currX - prevX, 2) + pow(currY - prevY, 2));
				}

				prevX = currX;
				prevY = currY;
			}

			curNumVehicle++;
			//Da er vi ferdig med en rute
			//og så avstanden tilbake;
			returnDistance = sqrt(pow(depotX - prevX, 2) + pow(depotY - prevY, 2));
			if (returnDistance > COMMUNICATION_RADIUS) {
				currentLength += returnDistance - COMMUNICATION_RADIUS;
			}
			


			if (currentLength > maxLength) {
				maxLength = currentLength;
			}
			sumLength += currentLength;
		}

		length = maxLength*maxLength + maxLength + sumLength / data->num_Depots;

		fitness = length;
		//fitness += PENALTY_LINEAR_VEHICLES * routes.size();
		if (curNumVehicle > 1) {
			cout << "Dette skal ikke skje \n";
			isInfeasable = 1;
			fitness += PENALTY_TO_MANY_VEHICLES;
		}
	}
	return length;
}

double individual::calc_differance(const individual& optimal) {
	if (!USE_CUSTOMMER_DIFFERENCE) {
		for (int d = 0; d != depoRoutes.size(); ++d) {
			for (int r = 0; r != depoRoutes[d].size(); ++r) {
				if (r < optimal.depoRoutes[d].size()) {
					if (depoRoutes[d][r] != optimal.depoRoutes[d][r]) {
						//+ for hver rute som er anderledes
						++difference;
					}
				}
				else {
					//+for hver vi er lengre en optimal
					++difference;
				}
			}
			//legg til antall ruter optimal er lengre enn oss
			if (optimal.depoRoutes[d].size() > depoRoutes[d].size()) {
				difference += optimal.depoRoutes[d].size() - depoRoutes[d].size();
			}
		}
	}
	else {
		for (int d = 0; d != depoRoutes.size(); ++d) {
			for (int r = 0; r != depoRoutes[d].size(); ++r) {
				if (r < optimal.depoRoutes[d].size()) {
					for (int c = 0; c < depoRoutes[d][r].size(); ++c) {
						if (c < optimal.depoRoutes[d][r].size()) {
							if (depoRoutes[d][r][c] != optimal.depoRoutes[d][r][c]) {
								difference++;
							}
						}
						else
						{
							difference++;
						}
					}
				}
				else {
					//+for hver vi er lengre en optimal
					difference += depoRoutes[d][r].size();
				}
			}
			//legg til antall ruter optimal er lengre enn oss
			if (optimal.depoRoutes[d].size() > depoRoutes[d].size()) {
				//dette blir ikke helt riktig men w/e. Tipper det er va 5custommers per rute
				difference += (optimal.depoRoutes[d].size() - depoRoutes[d].size())*5;
			}
		}
	}
	
	return difference;
}



bool makeRoutes(const vector<int>& depo, int depoId, vector< vector<int>>* routes, problemData* data) {
	/*routes->clear();
	auto it_begin = depo.begin();
	auto it_end = depo.end();*/

	routes->clear();
	routes->push_back(depo);
	return true;
	/*vector<int> route;
	
	while (it_begin != it_end) {
		route.push_back(*it_begin);

		if(!testFeasability(depoId, route, data)) {
			if (route.size() == 1) {
				//cout << "Infeasable point in depo";
				return 0;
				system("PAUSE");
			}

			route.pop_back();
			routes->push_back(route);
			route.clear();
		}
		else {
			it_begin++;
		}
	}
	if (route.size() > 0) {
		routes->push_back(route);
	}*/

	
	/*
	if (SIMPLE_MODE) return 1;
	//i simple mode hopper vi over resten

	//prøve å finn en mer optimal rute:

	double bestLength = routeLength(routes, depoId, data);
	double currentLength;
	bool feasable;

	bool doEnd = 0;
	
	//hvis vi ikke kunne flytte mer inn i den siste, så må vi slutte
	while (!doEnd && routes->size() > 1) {
		for (auto r = routes->end() - 1; r != routes->begin(); r--) {
			r->insert(r->begin(), *((r - 1)->end() - 1));
			feasable = testFeasability(depoId, *r, data);
			if (feasable) {
				(r - 1)->erase((r - 1)->end() - 1);
				currentLength = routeLength(routes, depoId, data);

				if (currentLength < bestLength) {
					bestLength = currentLength;
				}
				else {
					(r - 1)->insert((r - 1)->end(), *(r->begin()));
					r->erase(r->begin());
					if (r == routes->end() - 1) doEnd = 1;
					break;
				}

			}
			else {
				r->erase(r->begin());
				if (r == routes->end() - 1) doEnd = 1;
				break;
			}
		}
	}
	return 1;*/
}

double routeLength(vector< vector<int>>* routes, int depoID, problemData* data) {
	double length = 0;

	double prevX = 0;
	double prevY = 0;
	double currX = 0;
	double currY = 0;

	double depotX = 0;
	double depotY = 0;

	depotX = data->depots[depoID].x;
	depotY = data->depots[depoID].y;

	for (auto r = routes->begin(); r != routes->end(); ++r) {
		prevX = depotX;
		prevY = depotY;
		for (auto c = r->begin(); c != r->end(); ++c) {
			currX = data->customers[*c].x;
			currY = data->customers[*c].y;

			length += sqrt(pow(currX - prevX, 2) + pow(currY - prevY, 2));

			prevX = currX;
			prevY = currY;
		}
		//Da er vi ferdig med en rute
		//og så avstanden tilbake;
		length += sqrt(pow(depotX - prevX, 2) + pow(depotY - prevY, 2));
	}

	return length;
}

void calcTimeLoad(int depotID, const vector<int>& route, problemData* data, double* time, double* load) {
	*time = 0;
	*load = 0;

	double distance = 0;
	double extraDuration = 0;
	double prevX = 0;
	double prevY = 0;
	double currX = 0;
	double currY = 0;

	double depotX = data->depots[depotID].x;
	double depotY = data->depots[depotID].y;

	prevX = depotX;
	prevY = depotY;

	for (auto c = route.begin(); c != route.end(); ++c) {
		currX = data->customers[*c].x;
		currY = data->customers[*c].y;

		distance += sqrt(pow(currX - prevX, 2) + pow(currY - prevY, 2));
		*load += data->customers[*c].demand;
		extraDuration += data->customers[*c].service_duration;

		prevX = currX;
		prevY = currY;
	}
	distance += sqrt(pow(depotX - prevX, 2) + pow(depotY - prevY, 2));

	//beregne fiteness for denne ruta
	*time = distance + extraDuration;
}


bool testFeasability(int depotID, const vector<int>& route, problemData* data) {
	double max_load = data->depots[depotID].max_load;
	double max_duration = data->depots[depotID].max_duration;

	double curNumVehicle = 0;
	double currentTime = 0;
	double currentLoad = 0;
	double currentDistance = 0;
	double extraDuration = 0;
	double prevX = 0;
	double prevY = 0;
	double currX = 0;
	double currY = 0;

	double depotX = data->depots[depotID].x;
	double depotY = data->depots[depotID].y;

	prevX = depotX;
	prevY = depotY;

	for (auto c = route.begin(); c != route.end(); ++c) {
		currX = data->customers[*c].x;
		currY = data->customers[*c].y;

		currentDistance += sqrt(pow(currX - prevX, 2) + pow(currY - prevY, 2));
		currentLoad += data->customers[*c].demand;
		extraDuration += data->customers[*c].service_duration;

		prevX = currX;
		prevY = currY;
	}
	currentDistance += sqrt(pow(depotX - prevX, 2) + pow(depotY - prevY, 2));

	//beregne fiteness for denne ruta
	currentTime = currentDistance + extraDuration;

	if (max_duration != 0 && currentTime > max_duration) {
		return 0;
	}
	if (max_load != 0 && currentLoad > max_load) {
		return 0;
	}
	return 1;
}