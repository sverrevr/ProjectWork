#pragma once

#include "problemData.h"
#include "Parameters.h"
#include <vector>


class individual {
public:
	problemData* data;

	//depo[0] er depo 0 sine kunder i rekkefølgen de blir ekspedert i
	//Nå tester vi med å legge inn 0 mellom rutene.
	//Ex: 123045060 = [1 2 3] [4 5] [6]
	std::vector< std::vector<int>> depos;

	std::vector< std::vector< std::vector<int>>> depoRoutes;

	bool isInfeasable = 1;


	double fitness = INFINITY;
	double length = INFINITY;
	double difference = 0;
	double normalizationFactor = 0;
	double numClones = 0;

	double FAILED = 0;

	individual(problemData* data);
	individual() {}
	double calc_fitness(); //lower is better
	double calc_differance(const individual& optimal);
	void normalize_differance(double faktor) { normalizationFactor = faktor; }
	bool operator<(const individual& rhs) { 
		return fitness-difference*normalizationFactor +CLONE_PENALTY*numClones + 10000*FAILED < rhs.fitness-rhs.difference*normalizationFactor + CLONE_PENALTY*rhs.numClones;
	}
};

struct pos {
	double x;
	double y;
};

bool makeRoutes(const std::vector<int>& depo, int depoId, std::vector< std::vector<int>>* routes, problemData* data);
bool testFeasability(int depotID, const std::vector<int>& route, problemData* data);
double routeLength(std::vector< std::vector<int>>* routes, int depoID, problemData* data);
void calcTimeLoad(int depotID, const std::vector<int>& route, problemData* data, double* time, double* load);