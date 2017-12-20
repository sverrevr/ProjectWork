#pragma once
#include <string>
#include "problemData.h"
#include "individual.h"

void load(std::string filename, problemData* data);
void save(std::string filename, individual* solution);