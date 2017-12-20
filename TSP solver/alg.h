#pragma once
#include <vector>
#include "individual.h"

double custommerDepoDistance(const customerData& c, const depotData& d);
double custommerPosDistance(const customerData& c, const pos& p);
double custommerCustommerDistance(const customerData& c, const customerData& d);
void rearange_list_of_possitive_numbers(std::vector<int>* vec, int move_location_begin, int move_location_end, int paste_location);

void testIntegrity(individual* ind);