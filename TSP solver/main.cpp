#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <fstream>

#include "main.h"

using namespace std;



int main() {
	srand(time(NULL));

	string fileRead = "Data/p" + (string)FILE_ID;
	string fileWrite = "Data/s" + (string)FILE_ID + ".res";

	problemData data;
	load(fileRead, &data);

	individual alpha = lifecycle(&data);
	save(fileWrite, &alpha);

	cout << "Is feasable: " << !alpha.isInfeasable << endl;
	cout << "Fitness: " << alpha.length << endl;
	system("PAUSE");

	return 0;
}

void mutate(individual* ind) {
	int depo_id = rand() % ind->depos.size();		//velg ett depo
	vector<int>* depo =  &(ind->depos[depo_id]);	//peker til det depoet
	
	//Move
	if (MUTATION_INTRA_MOVE && !(rand() % MUTATION_INTRA_MOVE) && depo->size()>1) {
		int pos1 = rand() % depo->size();
		int pos2 = rand() % depo->size();
		if (pos1 != pos2) {
			rearange_list_of_possitive_numbers(depo, pos1, pos1 + 1, pos2);
		}
	}

	//swap
	if (MUTATION_SWAP && !(rand() % MUTATION_SWAP) && depo->size()>1) {
		int pos1 = rand() % depo->size();
		int pos2 = rand() % depo->size();
		int val1 = depo->at(pos1);
		depo->at(pos1) = depo->at(pos2);
		depo->at(pos2) = val1;
	}

	//Move section
	if (MUTATION_MOVE_SECTION && !(rand() % MUTATION_MOVE_SECTION) && depo->size()>1) {
		int pos1 = rand() % depo->size();
		int pos2 = rand() % depo->size();
		int pos3 = rand() % depo->size();
		rearange_list_of_possitive_numbers(depo, min(pos1,pos2), max(pos1, pos2), pos3);
	}


	//Reverse
	if (MUTATION_REVERSE && !(rand() % MUTATION_REVERSE) && depo->size() >1) {
		int pos1 = rand() % depo->size();
		int pos2 = rand() % depo->size();
		if (pos1 != pos2) {
			reverse(depo->begin() + min(pos1,pos2), depo->begin() + max(pos1,pos2));
		}
	}

	//move to optimal point	
	if (MUTATION_OPTIMAL_MOVE && !(rand() % MUTATION_OPTIMAL_MOVE) && depo->size()>0) {
		int cID = rand() % depo->size();
		int custommer = depo->at(cID);
		depo->erase(depo->begin() + cID);
		int optimalId = findOptimalPlacement(*depo, custommer, ind->data, depo_id);
		depo->insert(depo->begin() + optimalId, custommer);
	}



	/*Inter depo*/
	//move one
	if (MUTATION_INTER_MOVE && !(rand()%MUTATION_INTER_MOVE) && depo->size()>0) {
		int depo2_id = rand() % ind->depos.size();
		vector<int>* depo2 = &(ind->depos[depo2_id]);
		if (depo_id != depo2_id && depo2->size() >0) {
			//Move
			int pos1 = rand() % depo->size();
			auto it_pos1 = depo->begin() + pos1;
			double distDepo1 = custommerDepoDistance(ind->data->customers[(*depo)[pos1]], ind->data->depots[depo_id]);
			double distDepo2 = custommerDepoDistance(ind->data->customers[(*depo)[pos1]], ind->data->depots[depo2_id]);
			if (distDepo2 < distDepo1 * MAX_INTER_MOVE_RELLATIVE_DIFF) {
				int pos2 = rand() % (depo2->size());
				depo2->insert(depo2->begin() + pos2, *(depo->begin() + pos1));
				depo->erase(depo->begin() + pos1);
			}
		}
	}	

	//move section
	if (MUTATION_INTER_MOVE_SECTION && !(rand() % MUTATION_INTER_MOVE_SECTION) && depo->size()>0) {
		int depo2_id = rand() % ind->depos.size();
		vector<int>* depo2 = &(ind->depos[depo2_id]);
		if (depo_id != depo2_id && depo2->size() >0) {
			//Move
			int pos1_1 = rand() % depo->size();
			int pos1_2 = rand() % depo->size();
			auto it_pos1_1 = depo->begin() + min(pos1_1,pos1_2);
			auto it_pos1_2 = depo->begin() + max(pos1_1, pos1_2);

			bool doIt = 1;
			for (auto it = it_pos1_1; it != it_pos1_2; ++it) {
				double distDepo1 = custommerDepoDistance(ind->data->customers[(*depo)[min(pos1_1,pos1_2)]], ind->data->depots[depo_id]);
				double distDepo2 = custommerDepoDistance(ind->data->customers[(*depo)[max(pos1_1, pos1_2)]], ind->data->depots[depo2_id]);
				if (distDepo2 >= distDepo1 * MAX_INTER_MOVE_RELLATIVE_DIFF) doIt = 0;
			}
			if (doIt) {
				int pos2 = rand() % (depo2->size());
				depo2->insert(depo2->begin() + pos2, it_pos1_1, it_pos1_2);
				depo->erase(it_pos1_1, it_pos1_2);
			}
		}
	}

	//swap
	if (MUTATION_INTER_SWAP && !(rand() % MUTATION_INTER_SWAP) && depo->size() > 0) {
		int depo2_id = rand() % ind->depos.size();
		vector<int>* depo2 = &(ind->depos[depo2_id]);
		if (depo_id != depo2_id && depo2->size() > 0) {
			int loc1 = rand() % depo->size();
			int loc2 = rand() % depo2->size();
			int temp = depo->at(loc1);
			depo->at(loc1) = depo2->at(loc2);
			depo2->at(loc2) = temp;
		}
	}
}

//Nb c should be removed from depo
int findOptimalPlacement(const vector<int>& depo, int c, problemData* data, int depoID) {
	double bestVal = INFINITY;
	int bestLoc;

	double currentVal;

	int max_num_vehicle_per_route = data->num_Vehicles;
	int curNumVehicle;

	vector<int> copy;
	vector<vector<int>> routes;

	double prevX;
	double prevY;
	double currX;
	double currY;

	double depotX = data->depots[depoID].x;
	double depotY = data->depots[depoID].y;

	copy = depo;
	for (int i = 0; i < depo.size(); ++i) {
		copy.insert(copy.begin() + i, c);
		makeRoutes(depo, depoID, &routes, data);
		
		currentVal = 0;
		curNumVehicle = 0;

		for (auto r = routes.begin(); r != routes.end(); ++r) {
			prevX = depotX;
			prevY = depotY;
			for (auto c = r->begin(); c != r->end(); ++c) {
				currX = data->customers[*c].x;
				currY = data->customers[*c].y;

				currentVal += sqrt(pow(currX - prevX, 2) + pow(currY - prevY, 2));

				prevX = currX;
				prevY = currY;
			}

			curNumVehicle++;
			//Da er vi ferdig med en rute
			//og så avstanden tilbake;
			currentVal += sqrt(pow(depotX - prevX, 2) + pow(depotY - prevY, 2));
		}

		if (curNumVehicle > max_num_vehicle_per_route) {
			currentVal+= PENALTY_TO_MANY_VEHICLES;
		}

		if (currentVal < bestVal) {
			bestVal = currentVal;
			bestLoc = i;
		}
		//sletter den vi akkuratt la til
		copy.erase(copy.begin() + i);
	}
	return bestLoc;
}



void crossover(const individual& par1, const individual& par2, individual* child) {
	//HER ER DET EN BUG! depoRoutes og depos er ikke like på slutten av denne koden

	*child = par2;

	//int depo = rand() % par1.data->num_Depots;

	/*if (par1.depos[depo].size() == 0) {
		return;
	}*/

	//velger ett tilfeldig depo fra par1
	int depo = rand() % par1.depos.size();
	const vector<int>* route = &(par1.depos[depo]);

	bool doBreak = 0;

	//vector<int> routeElements = *route;

	//Må først gå gjennom barnet og slette alle elementene fra den ruta vi skal ta fra parent 1
	bool didErase;
	for (auto r = route->begin(); r != route->end(); ++r) {
		didErase = 0;
		for (auto d = child->depos.begin(); !didErase && d != child->depos.end(); ++d) {
			auto elem = find(d->begin(), d->end(), *r);
			//auto elem = d->end();
			if (elem != d->end()) {
				d->erase(elem);
				didErase = true;
			}
		}
	}
	//Ok nå har vi slettet alle tilfeller av elementene i den ruta hos barnet, nå må vi legge til ruta på en eller annet måte...
	//hva om vi prøver å plassere den på alle mulige steder og så putter den inn der det er best
	//dette blir n^2 kjøretid, men det går kanskje?
	

	int bestPos;
	double bestVal = INFINITY;
	int bestFeseable;
	double bestFeseableVal = INFINITY;
	int bestFeseableRouteLoc = 0;


	int currentPos;
	double currentVal;
	int currentFeseable;
	double currentFeseableVal;
	int currentRouteLocation = 0;


	//Skal plassere alle elementene fra den ruta vi ønsker å plassere
	for(auto c = route->begin(); c != route->end(); ++c) {

		//Går gjennom alle rutene hos barnet for å finne beste plassering

		caclulatedAddedWeight(child->depos[depo], depo, *c, child->data, &currentPos, &currentVal, &currentFeseable, &currentFeseableVal);
			
		if (currentVal < bestVal) {
			bestVal = currentVal;
			bestPos = currentPos;

		}
		if (currentFeseableVal < bestFeseableVal) {
			bestFeseableVal = currentFeseableVal;
			bestFeseable = currentFeseable;
		}
		child->depos[depo].insert(child->depos[depo].begin() + bestPos,*c);
	}

	child->depoRoutes.clear();
	child->depoRoutes.push_back(child->depos);
}

bool testIntegrity(vector<int>& depo, vector<vector<int>>& routes) {
	int j = 0;
	auto it = routes.begin();
	for (int i = 0; i < depo.size(); ++i) {
		if (depo[i] != it->at(j)) {
			return 0;
		}
		j++;
		if (j == it->size()) {
			it++;
			j = 0;
		}
	}
	return 1;
}

//plasseres før bestPossition
void caclulatedAddedWeight(const vector<int>& route, int depot, int customer, problemData* data, int* bestPos, double* bestVal, int* bestFesable, double* bestFesableVal) {
	if (route.size() == 0) {
		*bestPos = 0;
		*bestVal = 0;
		*bestFesable = 0;
		*bestFesableVal = 0;
		return;
	}
	*bestVal = INFINITY;
	*bestFesableVal = INFINITY;
	double currentWeight = 0;

	vector<int> copy;
	
	//Mellom depo og første
	//+ avstand depo->oss
	double usDepoDistance = custommerDepoDistance(data->customers[customer], data->depots[depot]);
	if (usDepoDistance > COMMUNICATION_RADIUS) {
		currentWeight += usDepoDistance;
		currentWeight -= COMMUNICATION_RADIUS;
	}
	//+ avstand oss -> kunde 0
	currentWeight += custommerCustommerDistance(data->customers[route[0]], data->customers[customer]);
	//- avstand kunde 0  -> depo
	double custDepoDist = custommerDepoDistance(data->customers[route[0]], data->depots[depot]);
	if (custDepoDist > COMMUNICATION_RADIUS) {
		currentWeight -= custommerDepoDistance(data->customers[route[0]], data->depots[depot]);
		currentWeight += COMMUNICATION_RADIUS;
	}

	if (currentWeight < *bestVal) {
		*bestVal = currentWeight;
		(*bestPos) = 0;
	}

	if (currentWeight < *bestFesableVal) {
		//tester om den er fesable;
		copy = route;
		copy.insert(copy.begin(), customer);
		if (testFeasability(depot, copy, data)) {
			*bestFesableVal = currentWeight;
			(*bestFesable) = 0;
		}
	}
	

	//mellom to kunder
	for (int i = 1; i < route.size(); ++i) {
		currentWeight = 0;
		//+ avstand kunde i-1->oss
		currentWeight += custommerCustommerDistance(data->customers[route[i-1]], data->customers[customer]); 
		//+ avstand oss -> kunde i
		currentWeight += custommerCustommerDistance(data->customers[route[i]], data->customers[customer]); 
		//- avstand kunde i-1  -> kunde i
		currentWeight -= custommerCustommerDistance(data->customers[route[i-1]], data->customers[route[i]]); 

		if (currentWeight < *bestVal) {
			*bestVal = currentWeight;
			(*bestPos) = i;
		}

		if (currentWeight < *bestFesableVal) {
			//tester om den er fesable;
			copy = route;
			copy.insert(copy.begin()+i, customer);
			if (testFeasability(depot, copy, data)) {
				*bestFesableVal = currentWeight;
				(*bestFesable) = i;
			}
		}

	}

	//mellom siste og depo
	currentWeight = 0;
	//+ avstand depo->oss
	usDepoDistance = custommerDepoDistance(data->customers[customer], data->depots[depot]);
	if (usDepoDistance > COMMUNICATION_RADIUS) {
		currentWeight += usDepoDistance;
		currentWeight -= COMMUNICATION_RADIUS;
	}	
	//+ avstand oss -> kunde siste
	currentWeight += custommerCustommerDistance(data->customers[route[route.size()-1]], data->customers[customer]);
	//- avstand kunde siste -> depo
	custDepoDist = custommerDepoDistance(data->customers[route[route.size() - 1]], data->depots[depot]);
	if (custDepoDist > COMMUNICATION_RADIUS) {
		currentWeight -= custDepoDist;
		currentWeight += COMMUNICATION_RADIUS;
	}


	if (currentWeight < *bestVal) {
		*bestVal = currentWeight;
		(*bestPos) = route.size();
	}

	//tester om den er fesable;
	if (currentWeight < *bestFesableVal) {
		//tester om den er fesable;
		copy = route;
		copy.insert(copy.end(), customer);
		if (testFeasability(depot, copy, data)) {
			*bestFesableVal = currentWeight;
			(*bestFesable) = route.size();
		}
	}

}

double elitism(vector<individual>& poppulation, vector<individual>* children ) {
	vector<int> takenAdults(0,0);

	double totalLowest = INFINITY;
	double lowestScore;
	int lowestId;
	for (int i = 0; i < ELITE_MEMBERS; ++i) {
		lowestScore = INFINITY;
		lowestId = 0;
		for (int j = 0; j < poppulation.size(); ++j) {
			if (poppulation[j].fitness < lowestScore) {
				if (find(takenAdults.begin(), takenAdults.end(), j) == takenAdults.end()) {
					lowestScore = poppulation[j].fitness;
					lowestId = j;
				}
			}
		}
		takenAdults.push_back(lowestId);
		(*children)[i] = poppulation[lowestId];
		if (lowestScore < totalLowest) {
			totalLowest = lowestScore;
		}
	}
	return totalLowest;
}


//Må oppdatere fitness først! Returns ID with lowest 
int tourney(vector<individual>* poppulation, individual* winner) {
	vector<int> participants(TOURNEY_SIZE, 0);

	//Forenkler til at den bare velger n tilfeldige uten å tenke på om den velger dobbelt
	//blir isåfall en kloning, så det er ike ett problem.
	/*
	vector<int> popId(poppulation->size(), 0);
	for (int i = 0; i < poppulation->size(); ++i) {
		popId[i] = i;
	}

	int randIndex;
	for (int i = 0; i < TOURNEY_SIZE; ++i) {
		randIndex = rand() % popId.size();
		participants[i] = popId[randIndex];
		popId.erase(popId.begin() + randIndex);
	}*/

	for (int i = 0; i < TOURNEY_SIZE; ++i) {
		participants[i] = rand() % POPPULATION_SIZE;
	}

	int lowestID;
	//sjangse for at en tilfeldig velges
	if (TOURNEY_ERROR_RATE && (rand() % TOURNEY_ERROR_RATE)) {
		lowestID = participants[rand() % participants.size()];
	}
	else {
		//Den beste velges istedet
		double lowestFitness = INFINITY;

		double currentFitness;
		for (int i = 0; i < TOURNEY_SIZE; ++i) {
			currentFitness = (*poppulation)[participants[i]].fitness;
			if (currentFitness < lowestFitness) {
				lowestFitness = currentFitness;
				lowestID = participants[i];
			}
		}
	}
	*winner = (*poppulation)[lowestID];
	return lowestID;
}

individual lifecycle(problemData* data) {
	time_t t1, t2;
	t1 = clock();
	//init poppulasjon
	vector<individual> mixPoppulation(2*POPPULATION_SIZE, individual(data));
	for (int i = 0; i < POPPULATION_SIZE; ++i) {
		mixPoppulation[i] = individual(data);
		mixPoppulation[i].calc_fitness();
	}
	sort(mixPoppulation.begin(), mixPoppulation.begin()+POPPULATION_SIZE);


	int childIndex = 0;
	individual parent1;
	individual parent2;
	individual child(data);

	double bestFitness;
	int bestFitnessID;

	int depo_id;
	int route_id;

	double maxDifference;
	double currentDifference;
	double normalizationFactor;

	int numCopies;

	int maxGenerations = GENERATION_CYCLES;

	double prevBestValue = INFINITY;

	for (int i = 0; i < maxGenerations; ++i) { 
		childIndex = POPPULATION_SIZE;
		for (; childIndex < 2*POPPULATION_SIZE; ++childIndex) {
			tourney(&mixPoppulation, &parent1);
			if (CLONE_RATE && !(rand() % CLONE_RATE)) {
				child = parent1;
			}
			else {
				tourney(&mixPoppulation, &parent2);
				crossover(parent1, parent2, &child);
			}
			//må mutere etterpå slik at vi slipper å regne ut ruter på nytt
			mutate(&child);
			mixPoppulation[childIndex] = child;
		}


		//beregner fitnessen til barna sånn at vi kan sortere
		for (auto it = mixPoppulation.begin() + POPPULATION_SIZE; it != mixPoppulation.end(); ++it) {
			it->calc_fitness();
		}

		//finn beste og plasser først;
		bestFitness = mixPoppulation[0].fitness;
		bestFitnessID = 0;
		for (int i = 1; i < 2 * POPPULATION_SIZE; ++i) {
			if (mixPoppulation[i].fitness < bestFitness) {
				bestFitness = mixPoppulation[i].fitness;
				bestFitnessID = i;
			}
		}
		if (bestFitnessID != 0) {
			iter_swap(mixPoppulation.begin(), mixPoppulation.begin() + bestFitnessID);
		}


		//beregn forskjellen mellom listene
		maxDifference = 0;
		if (USE_SIMPLE_DIFFERENCE) {
			for (auto it = mixPoppulation.begin()+1; it != mixPoppulation.end(); ++it) {
				it->difference = 0;
				currentDifference = it->calc_differance(mixPoppulation[0]);
				maxDifference = max(maxDifference, currentDifference);
			}
		}
		else {
			for (auto it = mixPoppulation.begin(); it != mixPoppulation.end(); ++it) {
				it->difference = 0;
				for (auto jt = mixPoppulation.begin(); jt != mixPoppulation.end(); ++jt) {
					if (it != jt) {
						it->calc_differance(*jt);
					}
				}
				currentDifference = it->difference;
				maxDifference = max(maxDifference, currentDifference);
			}
		}

		//normaliser forskjellen
		normalizationFactor = mixPoppulation[0].fitness / (maxDifference*DIFFERENCE_SCALING);
		for (auto it = mixPoppulation.begin() + 1; it != mixPoppulation.end(); ++it) {
			it->normalize_differance(normalizationFactor);
		}
		numCopies = 0;
		//tell opp kloner
		for (auto i = mixPoppulation.begin(); i != mixPoppulation.end()-1; ++i) {
			for (auto j = i + 1; j != mixPoppulation.end(); ++j) {
				if (i->depos == j->depos) {
					j->numClones++;
					numCopies++;
				}
			}
		}

		//sorter mhp avstand og forskjellen
		//gir ikke mening å sortere den forrige beste mhp dette, siden den ikke har fått trukket fra.
		//så sorterer ut resten, for deretter å finne den med best fitness (ikke forskjell) og plassere først
		sort(mixPoppulation.begin() + 1, mixPoppulation.end());

		//Randomizer den siste delen for å få med litt random
		random_shuffle(mixPoppulation.begin()+(POPPULATION_SIZE-RANDOM_SECTION), mixPoppulation.end());

		//legger til noen nye random individer
		for (auto it = mixPoppulation.begin() + (POPPULATION_SIZE - NEW_RANDOM_INDIVIDUALS); it != mixPoppulation.begin() + POPPULATION_SIZE; ++it) {
			*it = individual(data);
			it->calc_fitness();
		}
	

		bestFitness = mixPoppulation[0].length;
		if (!(i%PRINT_RATE)) {
			cout << i << ":\t" << bestFitness << "\t|\t" << numCopies << endl;
		}
		if (!(i%PRINT_RATE) && i != 0) {
			string fileWrite = "Data/s" + (string)FILE_ID + ".res";
			save(fileWrite, &mixPoppulation[0]);

			if (bestFitness < prevBestValue) {
				t2 = clock();
				float timeDiff = (float)t2 - (float)t1;
				float runTime = timeDiff / CLOCKS_PER_SEC;
				prevBestValue = bestFitness;
				fstream metadata("Data/s"+ (string)FILE_ID +"_tspMetadata.txt", fstream::out);
				metadata << "Run time: " << runTime << endl;
				metadata.close();
			}
		}

		if (i == maxGenerations-1) {
			string fileWrite = "Data/" + (string)FILE_ID + ".res";
			save(fileWrite, &mixPoppulation[0]);

			if (bestFitness < prevBestValue) {
				t2 = clock();
				float timeDiff = (float)t2 - (float)t1;
				float runTime = timeDiff / CLOCKS_PER_SEC;
				prevBestValue = bestFitness;
				fstream metadata("tspMetadata.txt", fstream::out);
				metadata << "Run time: " << runTime << endl;
				metadata.close();
			}


			cout << "How many more? ";
			unsigned int answer;
			cin >> answer;
			maxGenerations += answer;
		}
	}
	double lowestScore = mixPoppulation[0].length;
	return mixPoppulation[0];
}