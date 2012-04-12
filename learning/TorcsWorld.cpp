#include "TorcsWorld.h"


TorcsWorld::TorcsWorld(LearningInterface* interface_pointer): dp_torcs_interface(interface_pointer)
{
	continuousStates	= true ;
    continuousActions	= true ;

	stateDimension		= 13 ;
	actionDimension     = 2 ;

	d_end_of_ep = false;
	//initState();
}


TorcsWorld::~TorcsWorld(void)
{
	delete dp_state;
}

void TorcsWorld::initState()
{
	dp_state = new State;
	dp_state->continuous = true;
	dp_state->discrete = false;
	dp_state->stateDimension = stateDimension;
	dp_state->continuousState = new double[stateDimension];
}

double TorcsWorld::act( Action * action) 
{
	dp_action = action;
	return dp_torcs_interface->getReward(); //DIT GAAT VAST NIET GOED
}