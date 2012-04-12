#include "LearningInterface.h"

using namespace std;

LearningInterface::LearningInterface(void)
{
	initState();
	dp_world = new TorcsWorld(this);
	dp_algorithm = new Qlearning("..\source\learning\TorcsWorldCfg.txt", dp_world ) ;
}


LearningInterface::~LearningInterface(void)
{
	delete dp_current_state->continuousState;
	delete dp_current_state;
}


void LearningInterface::initState(){
	dp_current_state = new State;
	dp_current_state->continuous = true;
	dp_current_state->continuous = false;
	dp_current_state->stateDimension = 13;
	dp_current_state->continuousState = new double[13];
}


void LearningInterface::setState(vector<double>* features)
{
	for(size_t idx = 0; idx < features->size(); idx++){
		dp_current_state->continuousState[idx] = features->at(idx);
	}
	dp_world->setState(dp_current_state);
}

void LearningInterface::printState()
{
	cout << "Printing dimensions of State through LearningInterface.\n";

	for(size_t idx = 0; idx < dp_current_state->stateDimension; idx++)
	{
		cout << "Dimension " << idx << " : " << dp_current_state->continuousState[idx] << endl;
	}
}

void LearningInterface::computeReward(int new_dist) {
	d_reward = new_dist - d_last_dist;
	d_last_dist = new_dist;
}

vector<double> LearningInterface::getAction()
{
	vector<double> driver_action;
	Action* world_action = dp_world->getAction();
	if (world_action == NULL)
		cout << "ERROR: request for action, while action is empty!\n";
	else
	{
		size_t dim = world_action->actionDimension;
		for(size_t idx = 0; idx < dim; idx++)
		{
			driver_action.push_back(world_action->continuousAction[idx]);
		}
	}
	return driver_action;
}

void LearningInterface::experimentMainLoop() {
	//gekopieerd uit runExperiment
}