#include "TorcsWorld.h"
using namespace std;

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
}

TorcsWorld::TorcsWorld(Configuration config)
{
	m_config = config;
	mp_state = NULL;
	mp_action = NULL;

	m_reward = 0;
	m_end_of_ep = false;

	switch(config) {
		case QLEARNING:
			continuousStates	= true ;
			discreteStates		= false ;
			continuousActions	= false ;
			discreteActions		= true ;
			stateDimension		= 8;
			actionDimension		= 0;
			numberOfStates		= 0;
			numberOfActions		= 21;
			break;

		case CACLA:
			continuousStates	= true ;
			continuousActions	= true ;

			discreteStates		= false ;
			discreteActions		= false ;

			stateDimension		= 3;
			actionDimension		= 2;
			numberOfStates		= 0;
			numberOfActions		= 0;
			break;

		case BAS:
			continuousStates	= true ;
			continuousActions	= true ;

			discreteStates		= false ;
			discreteActions		= false ;

			stateDimension		= 8;
			actionDimension		= 2;
			numberOfStates		= 0;
			numberOfActions		= 0;
			break;

		case QOS5:
			continuousStates	= true ;
			discreteStates		= false ;
			continuousActions	= false ;
			discreteActions		= true ;
			stateDimension		= 8;
			actionDimension		= 0;
			numberOfStates		= 0;
			numberOfActions		= 5;
			break;

		case QOS3:
			continuousStates	= true ;
			discreteStates		= false ;
			continuousActions	= false ;
			discreteActions		= true ;
			stateDimension		= 8;
			actionDimension		= 0;
			numberOfStates		= 0;
			numberOfActions		= 3;
			break;

		case QSTEER5:
			continuousStates	= true ;
			discreteStates		= false ;
			continuousActions	= false ;
			discreteActions		= true ;
			stateDimension		= 8;
			actionDimension		= 0;
			numberOfStates		= 0;
			numberOfActions		= 5;
			break;

		case QSTEER3:
			continuousStates	= true ;
			discreteStates		= false ;
			continuousActions	= false ;
			discreteActions		= true ;
			stateDimension		= 8;
			actionDimension		= 0;
			numberOfStates		= 0;
			numberOfActions		= 3;
			break;
	}
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

void TorcsWorld::convertDiscreteAction(Action* action, double* torcs_action)
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
		switch(m_config)
		{
		/*
		case QOS5:
			switch(action->discreteAction)
			{
				case 0:
					torcs_action[0] = 0;
					torcs_action[1] = -1;
					break;

				case 1:
					torcs_action[0] = 0;
					torcs_action[1] = -0.5;
					break;

				case 2:
					torcs_action[0] = 0;
					torcs_action[1] = 0;
					break;

				case 3:
					torcs_action[0] = 0;
					torcs_action[1] = 0.5;
					break;

				case 4:
					torcs_action[0] = 0;
					torcs_action[1] = 1;
					break;
				default:
					cerr << "\nAction value = " << action->discreteAction << ". WAIT WHAT? How could this value occur?\n";
			}
			break;

		case QOS3:
			switch(action->discreteAction)
			{
				case 0:
					torcs_action[0] = 0;
					torcs_action[1] = 1;
					break;

				case 1:
					torcs_action[0] = 0;
					torcs_action[1] = 0;
					break;

				case 2:
					torcs_action[0] = 0;
					torcs_action[1] = -1;
					break;

				default:
					cerr << "\nAction value = " << action->discreteAction << ". WAIT WHAT? How could this value occur?\n";
			}
			break;

		case QSTEER5:
			switch(action->discreteAction)
			{
				case 0:
					torcs_action[0] = 0;
					torcs_action[1] = 0;
					break;

				case 1:
					torcs_action[0] = 1;
					torcs_action[1] = 0;
					break;

				case 2:
					torcs_action[0] = -1;
					torcs_action[1] = 0;
					break;

				case 3:
					torcs_action[0] = 0.5;
					torcs_action[1] = 0;
					break;

				case 4:
					torcs_action[0] = -0.5;
					torcs_action[1] = 0;
					break;

				default:
					cerr << "\nAction value = " << action->discreteAction << ". WAIT WHAT? How could this value occur?\n";
			}
			break;

			case QSTEER3:
			switch(action->discreteAction)
			{
				case 0:
					torcs_action[0] = 0;
					torcs_action[1] = 0;
					break;

				case 1:
					torcs_action[0] = 1;
					torcs_action[1] = 0;
					break;

				case 2:
					torcs_action[0] = -1;
					torcs_action[1] = 0;
					break;

				default:
					cerr << "\nAction value = " << action->discreteAction << ". WAIT WHAT? How could this value occur?\n";
			}
			break; */
		case QLEARNING:
			switch(action->discreteAction)
			{
				//Steer left
				case 0:
					torcs_action[0] = 1;
					torcs_action[1] = 1;
					break;

				case 1:
					torcs_action[0] = 1;
					torcs_action[1] = 0;
					break;

				case 2:
					torcs_action[0] = 1;
					torcs_action[1] = -1;
					break;

			//steer half left
				case 3:
					torcs_action[0] = 0.5;
					torcs_action[1] = 1;
					break;

				case 4:
					torcs_action[0] = 0.5;
					torcs_action[1] = 0;
					break;

				case 5:
					torcs_action[0] = 0.5;
					torcs_action[1] = -1;
					break;
			//steer straight
				case 6:
					torcs_action[0] = 0;
					torcs_action[1] = 1;
					break;
				case 7:
					torcs_action[0] = 0;
					torcs_action[1] = 0;
					break;

				case 8:
					torcs_action[0] = 0;
					torcs_action[1] = -1;
					break;
				//steer half right
				case 9:
					torcs_action[0] = -0.5;
					torcs_action[1] = -1;
					break;

				case 10:
					torcs_action[0] = -0.5;
					torcs_action[1] = 0;
					break;

				case 11:
					torcs_action[0] = -0.5;
					torcs_action[1] = 1;
					break;
				//steer right
				case 12:
					torcs_action[0] = -1;
					torcs_action[1] = -1;
					break;
				case 13:
					torcs_action[0] = -1;
					torcs_action[1] = 0;
					break;

				case 14:
					torcs_action[0] = -1;
					torcs_action[1] = 1;
					break;

				//steer little left
				case 15:
					torcs_action[0] = 0.1;
					torcs_action[1] = 1;
					break;

				case 16:
					torcs_action[0] = 0.1;
					torcs_action[1] = 0;
					break;

				case 17:
					torcs_action[0] = 0.1;
					torcs_action[1] = -1;
					break;
				//steer little right
				case 18:
					torcs_action[0] = -0.1;
					torcs_action[1] = 1;
					break;
				case 19:
					torcs_action[0] = -0.1;
					torcs_action[1] = 0;
					break;

				case 20:
					torcs_action[0] = -0.1;
					torcs_action[1] = -1;
					break;


					/*case QLEARNING:
			switch(action->discreteAction)
			{
				//Steer straight
				case 0:
					torcs_action[0] = 0;
					torcs_action[1] = 1;
					break;

				case 1:
					torcs_action[0] = 0;
					torcs_action[1] = 0;
					break;

				case 2:
					torcs_action[0] = 0;
					torcs_action[1] = -1;
					break;

			//steer right
				case 3:
					torcs_action[0] = -1;
					torcs_action[1] = 1;
					break;

				case 4:
					torcs_action[0] = -1;
					torcs_action[1] = 0;
					break;

				case 5:
					torcs_action[0] = -1;
					torcs_action[1] = -1;
					break;
			//steer left
				case 6:
					torcs_action[0] = 1;
					torcs_action[1] = 1;
					break;
				case 7:
					torcs_action[0] = 1;
					torcs_action[1] = 0;
					break;

				case 8:
					torcs_action[0] = 1;
					torcs_action[1] = -1;
					break;
				//steer half right
				case 9:
					torcs_action[0] = -0.5;
					torcs_action[1] = -1;
					break;

				case 10:
					torcs_action[0] = -0.5;
					torcs_action[1] = 0;
					break;

				case 11:
					torcs_action[0] = -0.5;
					torcs_action[1] = 1;
					break;
				//steer half left
				case 12:
					torcs_action[0] = 0.5;
					torcs_action[1] = -1;
					break;
				case 13:
					torcs_action[0] = 0.5;
					torcs_action[1] = 0;
					break;

				case 14:
					torcs_action[0] = 0.5;
					torcs_action[1] = 1;
					break;

				//steer little right
				case 15:
					torcs_action[0] = -0.05;
					torcs_action[1] = 1;
					break;

				case 16:
					torcs_action[0] = -0.05;
					torcs_action[1] = 0;
					break;

				case 17:
					torcs_action[0] = -0.05;
					torcs_action[1] = -1;
					break;
				//steer little left
				case 18:
					torcs_action[0] = 0.05;
					torcs_action[1] = 1;
					break;
				case 19:
					torcs_action[0] = 0.05;
					torcs_action[1] = 0;
					break;

				case 20:
					torcs_action[0] = 0.05;
					torcs_action[1] = -1;
					break;*/
				default:
					cerr << "\nAction value = " << action->discreteAction << ". WAIT WHAT? How could this value occur?\n";
			}
			break;

		default:
			cerr << "Wrong discrete action configuration selected.\n";
		}
	} else {
		cerr << "Wrong action input. Setting action pointer to NULL.\n";
		torcs_action = NULL;
	}
}

void TorcsWorld::convertContinuousAction(Action* action, double* torcs_action)
{
	//Get action[0] within boundaries of -1 and 1
	//steer
	if(action->continuousAction[0] > 0)
		torcs_action[0] = min(action->continuousAction[0], 1.0);
	else
		torcs_action[0] = max(action->continuousAction[0], -1.0);

	//Get action[1] within boundaries of -1 and 1
	//accel
	if(action->continuousAction[1] > 0)
		torcs_action[1] = min(action->continuousAction[1], 1.0);
	else
		torcs_action[1] = max(action->continuousAction[1], -1.0);
}