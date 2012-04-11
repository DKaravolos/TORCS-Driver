#include "LearningInterface.h"

using namespace std;

LearningInterface::LearningInterface(void)
{
	initState();
}


LearningInterface::~LearningInterface(void)
{
	delete dp_current_state->continuousState;
	delete dp_current_state;
}


void LearningInterface::initState(){
	dp_current_state = new State;
	dp_current_state->continuous = true;
	dp_current_state->stateDimension = 13;
	dp_current_state->continuousState = new double[13];
}


void LearningInterface::setState(vector<double>* features)
{
	for(size_t idx = 0; idx < features->size(); idx++){
		dp_current_state->continuousState[idx] = features->at(idx);
	}
}

void LearningInterface::printState()
{
	cout << "Printing dimensions of State through LearningInterface.\n";

	for(size_t idx = 0; idx < dp_current_state->stateDimension; idx++)
	{
		cout << "Dimension " << idx << " : " << dp_current_state->continuousState[idx] << endl;
	}
}
