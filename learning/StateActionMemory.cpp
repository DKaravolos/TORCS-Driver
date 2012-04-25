#include "StateActionMemory.h"

using namespace std;
StateActionMemory::StateActionMemory()
{
	mp_states = new vector<State>();
	mp_actions = new vector<Action>();
	mp_rewards = new vector<double>();
	mp_next_states = new vector<State>();
}

StateActionMemory::~StateActionMemory()
{
	delete mp_states;
	delete mp_actions;
	delete mp_rewards;
	delete mp_next_states;
}

void StateActionMemory::storeTuple(State* state, Action* action, double reward, State* next_state)
{
	if(state == NULL)
		throw std::invalid_argument("Cannot add NULL state to memory!");
	else if(action == NULL)
		throw std::invalid_argument("Cannot add NULL action to memory!");
	else if(next_state == NULL)
		throw std::invalid_argument("Cannot add NULL next_state to memory!");
	else
	{
		mp_states->push_back(*state);
		mp_actions->push_back(*action);
		mp_rewards->push_back(reward);
		mp_next_states->push_back(*next_state);
	}
}

void StateActionMemory::retrieveTupleAt(int index, State* state, Action* action, double& reward, State* next_state)
{
	//You would want pointers to constant objects here, but functions from rlcpp cannot handle this. 
	state = &mp_states->at(index);
	action = &mp_actions->at(index);
	reward = mp_rewards->at(index);
	next_state = &mp_next_states->at(index);
}

void StateActionMemory::printTuple(int index)
{
	State* state = &mp_states->at(index);
	Action* action = &mp_actions->at(index);
	State* next_state = &mp_next_states->at(index);
	bool cont_s = state->continuous;
	bool cont_a = action->continuous;
	cout << "Tuple "<<index << " : ";
	//print state
	if(cont_s)
		printf("state[1] = %.4f", state->continuousState[1]);
	else
		cout << "state = " << state->discreteState;
	
	//print action
	if(cont_a)
		printf("\taction[0] = %.4f", action->continuousAction[0]);
	else
		cout << "\taction = " << action->discreteAction;
	
	//print reward
	printf("\treward: %.4f",mp_rewards->at(index));
	
	//print next state
	if(cont_s)
		printf("\tnext_state[1] = %.4f", next_state->continuousState[1]);
	else
		cout << "\tnext_state = " << state->discreteState;
	cout << endl;
}

void StateActionMemory::printHead(int number)
{
	for(int idx = 0; idx < number; idx++)
	{
		printTuple(idx);
	}
}