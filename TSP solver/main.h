#pragma once

#include <vector>
#include "problemData.h"
#include "file.h"
#include "individual.h"
#include "Parameters.h"
#include "alg.h"

void mutate(individual* ind);
void crossover(const individual& par1, const individual& par2, individual* child);
int tourney(std::vector<individual>* poppulation, individual* winner);
individual lifecycle(problemData* data);
double elitism(std::vector<individual>& poppulation, std::vector<individual>* children);
void caclulatedAddedWeight(const std::vector<int>& route, int depot, int customer, problemData* data, int* bestPos, double* bestVal, int* bestFesable, double* bestFesableVal);
bool testIntegrity(std::vector<int>& depo, std::vector<std::vector<int>>& routes);
int findOptimalPlacement(const std::vector<int>& depo, int c, problemData* data, int depoID);