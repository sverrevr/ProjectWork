#pragma once
#include "stateClasses.h"
#include <vector>

enum MessageType {msg_noMessage, msg_reMakePath, msg_proposeNode, msg_pathProposal, msg_error, msg_newNode};
struct Message {
	MessageType type;
	int senderId;
	Vector2D sendPos; 
	
	//some form of structure that contains data

	std::vector<IcebergData>*  icebergData; //Gjør ikke noe om den endres innen den kommer frem

	Path path; //Skal være en kopi
	IcebergData newNode;
	proposedNode nodeProposal;
};