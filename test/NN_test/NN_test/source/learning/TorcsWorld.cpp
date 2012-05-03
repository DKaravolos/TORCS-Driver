#include "TorcsWorld.h"

TorcsWorld::TorcsWorld()
{
	continuousStates	= true ;
    discreteStates		= false ;
    continuousActions	= false ;
    discreteActions		= true ;
	stateDimension		= 13;
	actionDimension		= 0;
	numberOfStates		= 0;
	numberOfActions		= 15;

	mp_state = NULL;
	mp_action = NULL;

	m_reward = 0;
	m_end_of_ep = false;
	//initState();
}


TorcsWorld::~TorcsWorld(void)
{
	//delete mp_state;
}
/*
void TorcsWorld::initState()
{
	mp_state = new State;
	mp_state->continuous = true;
	mp_state->discrete = false;
	mp_state->stateDimension = stateDimension;
	mp_state->continuousState = new double[stateDimension];
}
*/
double TorcsWorld::act( Action * action) 
{
	//mp_action = action;
	//map de int van Action om naar een nuttige waarde voor TORCS

	return m_reward;
}

/*
convertAction could be changed into a void function, with a double* as input.
This means less chance of memory leak, but also less readability
*/
double* TorcsWorld::convertAction(Action* action)
{
	/*
	Discrete action conversion scheme.
	2 dimensions: acceleration (-1,0,1) and steer (-1,0,1) (or: left, middle, right)
	Mapping: (steer,accel)
	i.e.: 1 =  (-1,-1), 2 = (-1,0), 3 = (-1,1), ..., 9= (1,1)

	*/
	double* torcs_action = new double[2]; //This is deleted in MyFirstDriver.

	if(action->discrete && numberOfActions == 9)
	{
		switch(action->discreteAction)
		{
			case 0:
				torcs_action[0] = -1;
				torcs_action[1] = -1;
				break;

			case 1:
				torcs_action[0] = 0;
				torcs_action[1] = -1;
				break;

			case 2:
				torcs_action[0] = 1;
				torcs_action[1] = -1;
				break;

			case 3:
				torcs_action[0] = -1;
				torcs_action[1] = 0;
				break;

			case 4:
				torcs_action[0] = 0;
				torcs_action[1] = 0;
				break;

			case 5:
				torcs_action[0] = 1;
				torcs_action[1] = 0;
				break;

			case 6:
				torcs_action[0] = -1;
				torcs_action[1] = 1;
				break;

			case 7:
				torcs_action[0] = 0;
				torcs_action[1] = 1;
				break;

			case 8:
				torcs_action[0] = 1;
				torcs_action[1] = 1;
				break;
		}
		return torcs_action;
	}
	else if(action->discrete && numberOfActions == 15)
	{
		switch(action->discreteAction)
		{
			case 0:
				torcs_action[0] = -1;
				torcs_action[1] = -1;
				break;

			case 1:
				torcs_action[0] = -0.5;
				torcs_action[1] = -1;
				break;

			case 2:
				torcs_action[0] = 0;
				torcs_action[1] = -1;
				break;

			case 3:
				torcs_action[0] = 0.5;
				torcs_action[1] = -1;
				break;

			case 4:
				torcs_action[0] = 1;
				torcs_action[1] = -1;
				break;

			case 5:
				torcs_action[0] = -1;
				torcs_action[1] = 0;
				break;

			case 6:
				torcs_action[0] = -0.5;
				torcs_action[1] = 0;
				break;

			case 7:
				torcs_action[0] = 0;
				torcs_action[1] = 0;
				break;

			case 8:
				torcs_action[0] = 0.5;
				torcs_action[1] = 0;
				break;

			case 9:
				torcs_action[0] = 1;
				torcs_action[1] = 0;
				break;

			case 10:
				torcs_action[0] = -1;
				torcs_action[1] = 1;
				break;

			case 11:
				torcs_action[0] = -0.5;
				torcs_action[1] = 1;
				break;

			case 12:
				torcs_action[0] = 0;
				torcs_action[1] = 1;
				break;

			case 13:
				torcs_action[0] = 0.5;
				torcs_action[1] = 1;
				break;				
			
			case 14:
				torcs_action[0] = 1;
				torcs_action[1] = 1;
				break;
		}
		return torcs_action;
	}
	else
		return NULL;
}

void TorcsWorld::convertAction(Action* action, double* torcs_action)
{
	/*
	Discrete action conversion scheme.
	2 dimensions: acceleration (-1,0,1) and steer (-1,0,1) (or: left, middle, right)
	Mapping: (steer,accel)
	i.e.: 1 =  (-1,-1), 2 = (-1,0), 3 = (-1,1), ..., 9= (1,1)

	*/
	//double* torcs_action = new double[2]; //This is deleted in MyFirstDriver.
	mp_action = action;
	if(action->discrete)
	{
		switch(action->discreteAction)
		{
			case 0:
				torcs_action[0] = -1;
				torcs_action[1] = -1;
				break;
			case 1:
				torcs_action[0] = -1;
				torcs_action[1] = 0;
				break;
			case 2:
				torcs_action[0] = -1;
				torcs_action[1] = 1;
				break;
			case 3:
				torcs_action[0] = 0;
				torcs_action[1] = -1;
				break;
			case 4:
				torcs_action[0] = 0;
				torcs_action[1] = 0;
				break;
			case 5:
				torcs_action[0] = 0;
				torcs_action[1] = 1;
				break;
			case 6:
				torcs_action[0] = 1;
				torcs_action[1] = -1;
				break;
			case 7:
				torcs_action[0] = 1;
				torcs_action[1] = 0;
				break;
			case 8:
				torcs_action[0] = 1;
				torcs_action[1] = 1;
				break;
		}
	} else {
		std::cout << "Wrong action input. Setting action pointer to NULL" << std::endl;
		torcs_action = NULL;
	}
}