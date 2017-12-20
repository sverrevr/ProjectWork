#pragma once
#include <limits.h>
#include <vector>
#include "Definitions.h"


struct Vector2D{
	double x = 0;
	double y = 0;
	Vector2D(double x, double y) : x(x), y(y){}
	Vector2D() : x(0), y(0) {}
};

double distance2D(Vector2D lhs, Vector2D rhs);
Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs);
void normalize(Vector2D* vec);
Vector2D normalize(const Vector2D& vec);
void operator*=(Vector2D& lhs, double rhs);
Vector2D operator*(Vector2D& lhs, double rhs);
void operator+=(Vector2D& lhs, double rhs);
void operator+=(Vector2D& lhs, Vector2D& rhs);
Vector2D operator+(Vector2D& lhs, Vector2D& rhs);

struct State {
	Vector2D pos;
	Vector2D velocity;
};


struct PathNode {
	int iceberg = PATH_NO_ICEBERG;
	bool needsCom = false;
	bool getsCom = false;
	bool isComNode = false;
	Vector2D possition;
	double priority;
	//stuff with time cpnstraint and such
	bool operator==(const PathNode& rhs) const {
		return iceberg == rhs.iceberg;
	}
};

struct Path {
	int owner;
	int endIcebergId() const { return nodes.back().iceberg; }

	double objectiveValue;

	bool finished(Vector2D dronePos);
	int  currentGoal;
	PathNode& currentNode() { return nodes[currentGoal]; }
	PathNode& previousNode() { return nodes[currentGoal-1]; }
	void incrementCurrentGoal() { currentGoal++; }
	//std::vector<unsigned > icebergIds;

	std::vector<PathNode> nodes;

	bool allreadyHasAComNode = false;

	void reset(int droneId, Vector2D dronePossition);
	void reset();
	size_t size() { return nodes.size(); }
};

double lengthOfPath(Path* path);
Vector2D locationAtTimepoin(Path* path, double timepoint, int* lastNode);

//This struct is the data a drone has about the iceberg. Not the true state of the iceberg
struct IcebergData {
	//Constant states that never change for a node
	int id;
	
	//variables that should follow time since last planned
	int owner = ICEBERGDATA_NO_OWNER; //The drone that will solve this iceberg
									  //bool isVirtual = false; //It was a virtual bid that won this task.
									  //needsCommuncation?
	bool hasBeenBiddenOn = false;

	unsigned timeSincePathPlanned = 0;
	//can save data by not saving this one
	Path currentBestPath;


	//Vairables that should follow lowest time since last visit
	double priority = 0; //function of estimatedState and estimatedVariance;
	unsigned timeSinceLastVisit = 0;
	//Knowledge about the iceberg, is updated on communication
	State estimatedState;

	bool isComNode = false;
	int comNodeOwner = -1;
	Vector2D dronePos;
	Vector2D shipPos;
	double activeTime;
	double inactivationTime;
	


	//Functions

	void uppdatePriority(State shipState);
	

	bool operator<(const IcebergData& rhs) {
		return priority < rhs.priority;
	}

};


struct proposedNode {
	int nodeId;
	double priority;
	bool operator <(const proposedNode& rhs) const {
		if (priority != rhs.priority) {
			return priority < rhs.priority;
		}
		else {
			return nodeId < rhs.nodeId;
		}
	}
};