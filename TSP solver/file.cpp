#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include "file.h"

using namespace std;


void load(string filename, problemData *data) {

	fstream file(filename, fstream::in);

	if (!file.is_open()) {
		cout << "Could not open file";
		return;
	}

	string line;
	stringstream ss_line;
	getline(file, line);
	ss_line << line;
	ss_line >> data->num_Vehicles;
	ss_line >> data->num_Customers;
	ss_line >> data->num_Depots;

	ss_line.str(string());
	ss_line.clear();

	depotData depot;
	for (int i = 0; i < data->num_Depots; ++i) {
		getline(file, line);
		ss_line << line;
		ss_line >> depot.max_duration;
		ss_line >> depot.max_load;
		
		ss_line.str(string());
		ss_line.clear();

		data->depots.push_back(depot);
	}

	customerData customer;
	customer.id = -1;
	//Add an empty element to begin with to make it one-indexed
	data->customers.push_back(customer);
	for (int i = 0; i < data->num_Customers; ++i) {
		getline(file, line);
		ss_line << line;
		ss_line >> customer.id;
		ss_line >> customer.x;
		ss_line >> customer.y;
		ss_line >> customer.service_duration;
		ss_line >> customer.demand;
		
		ss_line.str(string());
		ss_line.clear();

		data->customers.push_back(customer);
	}

	for (int i = 0; i < data->num_Depots; ++i) {
		getline(file, line);
		ss_line << line;
		ss_line >> data->depots[i].id;
		ss_line >> data->depots[i].x;
		ss_line >> data->depots[i].y;

		ss_line.str(string());
		ss_line.clear();
	}

	file.close();
}


void save(string filename, individual* solution) {
	fstream file(filename, fstream::out);

	solution->calc_fitness();

	if (!file.is_open()) {
		cout << "Could not open file";
		return;
	}

	file << solution->length << endl;
	
	int vehicle_id = 1;
	int depoID = 0;

	vector<vector<int>> routes;
	double time;
	double load;

	for (int d = 0; d != solution->depos.size();++d) {
		vehicle_id = 1;
		depoID++; //depoID er en større enn d pga indeksering

		makeRoutes(solution->depos[d], d, &routes, solution->data);

		for (auto r = routes.begin(); r != routes.end(); ++r) {
			calcTimeLoad(d,*r,solution->data,&time,&load);
			file << depoID << ' ' << vehicle_id << ' ' << time << ' ' << load << " 0 ";
			for (auto c = r->begin(); c != r->end(); ++c) {
				file << *c << ' ';
			}
			file << "0\n";
			vehicle_id++;
		}
	}

	file.close();
}