#include "stateClasses.h"
#include <algorithm>
using namespace std;

void IcebergData::uppdatePriority(State shipState) {
	double dist = distance2D(shipState.pos, estimatedState.pos);
	if (dist > MIN_ICEBERG_DISTANCE) {
		priority = timeSinceLastVisit;
		priority *= max(1 - dist / MAX_ICEBERG_DISTANCE, 0.0);
	}
	else {
		priority = 0;
	}

	//! TODO
	//Take movements direction and speed of iceberg relative to ship into account
}

void Path::reset(int droneId) {
	owner = droneId;
	currentGoal = 0;
	nodes.clear();
}

void Path::reset() {
	owner = NO_OWNER;
	currentGoal = 0;
	nodes.clear();
}

bool Path::finished(Vector2D dronePos) {
	return (currentGoal >= nodes.size());
}


double distance2D(Vector2D lhs, Vector2D rhs) {
	double x_dif = lhs.x - rhs.x;
	double y_dif = lhs.y - rhs.y;

	return sqrt(pow(x_dif, 2) + pow(y_dif, 2));
}

Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs) {
	Vector2D returVec;
	returVec.x = lhs.x - rhs.x;
	returVec.y = lhs.y - rhs.y;
	return returVec;
}

void normalize(Vector2D* vec) {
	double length = sqrt(pow(vec->x, 2) + pow(vec->y, 2));
	if (length <= 1e-3) return;
	vec->x /= length;
	vec->y /= length;
}

Vector2D normalize(const Vector2D& vec) {
	Vector2D returVec = vec;
	normalize(&returVec);
	return returVec;
}

void operator*=(Vector2D& lhs, double rhs) {
	lhs.x *= rhs;
	lhs.y *= rhs;
}

Vector2D operator*(Vector2D& lhs, double rhs) {
	Vector2D newVec = lhs;
	newVec *= rhs;
	return newVec;
}


void operator+=(Vector2D& lhs, double rhs) {
	lhs.x += rhs;
	lhs.y += rhs;
}

void operator+=(Vector2D& lhs, Vector2D& rhs) {
	lhs.x += rhs.x;
	lhs.y += rhs.y;
}

Vector2D operator+(Vector2D& lhs, Vector2D& rhs) {
	Vector2D retVal = lhs;
	retVal += rhs;
	return retVal;
}

double lengthOfPath(Path* path) {
	double length = 0;
	auto jt = path->nodes.begin() + 1;
	for (auto it = path->nodes.begin(); jt != path->nodes.end(); ++it) {
		length += distance2D(it->possition, jt->possition);
		jt++;
	}
	return length;
}

