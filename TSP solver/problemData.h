#pragma once
#include <vector>

struct depotData {
	double max_duration; //0 == no constraint
	double max_load; //per vehicle

	int id;
	double x;
	double y;
};

struct customerData {
	int id;
	double x;
	double y;
	double service_duration; //0 == no constraing
	double demand; //weight demand
};

struct problemData {
	int num_Vehicles = 0; //per depot
	int num_Customers = 0;
	int num_Depots = 0;
	std::vector<depotData> depots;
	std::vector<customerData> customers; //starts at element 1, aka begin()+1
};

