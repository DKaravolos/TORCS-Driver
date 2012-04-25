#ifndef STATE_ACTION_MEMORY_H_
#define STATE_ACTION_MEMORY_H_
#include <iostream>
#include <vector>

#include "..\rlcpp\State.h"
#include "..\rlcpp\Action.h"

class StateActionMemory
{
public:
	StateActionMemory();
	~StateActionMemory();

	//LI functions
	void storeTuple(State* state, Action* action, double reward, State* next_state);
	void retrieveTupleAt(int idx, State* state, Action* action, double& reward, State* next_state);
	inline int getSize(){ return mp_states->size();}

	//print functions
	void printTuple(int index);
	void printHead(int number);

protected:
	std::vector<State>* mp_states;
	std::vector<Action>* mp_actions;
	std::vector<double>* mp_rewards;
	std::vector<State>* mp_next_states;

};

#endif //STATE_ACTION_MEMORY_H