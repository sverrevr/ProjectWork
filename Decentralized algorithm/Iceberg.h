#pragma once
#include "Definitions.h"
#include "stateClasses.h"

class Iceberg
{
public:
	State state;
	Iceberg();
	~Iceberg();
	void run();
};

