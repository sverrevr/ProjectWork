#pragma once
#include <vector>
#include <limits.h>

struct Edge {
	int source;
	int destination;
	double weight;
	Edge(int source, int destination, double weight) : source(source), destination(destination), weight(weight) {}
};

bool bellmanFord(const std::vector<Edge>& edges, int numVertecies, int source, int destination, std::vector<int>* bestPath, double* pathLength);