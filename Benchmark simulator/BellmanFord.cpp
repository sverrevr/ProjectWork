

//Based on wikipedias implementation of bellman ford
//https://en.wikipedia.org/wiki/Bellman%E2%80%93Ford_algorithm
//Date: 4. oct. 2017


#include "BellmanFord.h"
#include "Definitions.h"

using namespace std;


bool bellmanFord(const vector<Edge>& edges, int numVertecies, int source, int destination, vector<int>* bestPath, double* pathLength) {

	double* shortestDistanceToNode = new double[numVertecies];
	int* bestPredecessor = new int[numVertecies];

	for (int i = 0; i < numVertecies; ++i) {
		shortestDistanceToNode[i] = INFINITY;
		bestPredecessor[i] = -1;
	}
	shortestDistanceToNode[source] = 0;

	bool wasChange;
	for (int i = 0; i < numVertecies; ++i) {
		wasChange = false;
		for (int j = 0; j < edges.size(); ++j) {
			int s = edges[j].source;
			int d = edges[j].destination;
			double w = edges[j].weight;
			if (shortestDistanceToNode[s] + w < shortestDistanceToNode[d]) {
				shortestDistanceToNode[d] = shortestDistanceToNode[s] + w;
				bestPredecessor[d] = s;
				wasChange = true;
			}
		}
		if (!wasChange) break;
	}

	if (wasChange) {
		for (int j = 0; j < edges.size(); ++j) {
			int s = edges[j].source;
			int d = edges[j].destination;
			double w = edges[j].weight;
			if (shortestDistanceToNode[s] + w < shortestDistanceToNode[d]) {
				//there was a negative cycle
				return false;
			}
		}
	}

	int currentNode = destination;
	while (currentNode != source) {
		bestPath->insert(bestPath->begin(), currentNode);
		currentNode = bestPredecessor[currentNode];
		if (currentNode < 0 || currentNode > numVertecies || bestPath->size() > NUMBER_OF_ICEBERGS) {
			return false;
		}
	}
	bestPath->insert(bestPath->begin(), source);

	(*pathLength) = shortestDistanceToNode[destination];

	delete[] shortestDistanceToNode;
	delete[] bestPredecessor;

	return true;
}