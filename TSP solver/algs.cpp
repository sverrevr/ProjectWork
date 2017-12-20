#include "alg.h"
#include <iostream>
using namespace std;
//Moves [move_location_begin, move_location_end> to before paste_location.
void rearange_list_of_possitive_numbers(vector<int>* vec, int move_location_begin, int move_location_end, int paste_location) {
	auto m_b = vec->begin() + move_location_begin;
	auto m_e = vec->begin() + move_location_end;

	auto p_b = vec->begin() + paste_location;

	vector<int> copy;
	copy.insert(copy.begin(), m_b, m_e);

	for (auto it = m_b; it != m_e; ++it) {
		*it = -1;
	}

	vec->insert(p_b, copy.begin(), copy.end());

	for (auto it = vec->begin(); it != vec->end();) {
		if (*it == -1) {
			it = vec->erase(it);
		}
		else {
			++it;
		}

	}
}


double custommerDepoDistance(const customerData& c, const depotData& d) {
	return sqrt(pow(c.x - d.x, 2) + pow(c.y - d.y, 2));
}
double custommerPosDistance(const customerData& c, const pos& p) {
	return sqrt(pow(c.x - p.x, 2) + pow(c.y - p.y, 2));
}
double custommerCustommerDistance(const customerData& c, const customerData& d) {
	return sqrt(pow(c.x - d.x, 2) + pow(c.y - d.y, 2));
}

void testIntegrity(individual* ind) {
	problemData* data = ind->data;

	vector<int> custId(data->num_Customers +1 ,0);


	for (auto d = ind->depos.begin(); d != ind->depos.end(); ++d) {
		for (auto c = d->begin(); c != d->end(); ++c) {
			custId[*c]++;
		}
	}

	for (auto it = custId.begin() + 1; it != custId.end(); ++it) {
		if (*it != 1) {
			cout << "integrity broken\n";
		}
	}
}