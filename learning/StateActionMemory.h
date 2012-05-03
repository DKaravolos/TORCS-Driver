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
	void storeTuple	(State* state, Action* action, double reward, State* next_state, 
					bool end_of_ep, double td_error);

	void retrieveTupleAt(int idx, State* state, Action* action, double& reward, 
						State* next_state, bool& end_of_ep);
	void retrieveTupleAt(int idx, State* state, Action* action, double& reward, 
						State* next_state, bool& end_of_ep, double& td_error);
	inline int getSize(){ return mp_states->size();}

	//print functions
	void printTuple(int index);
	void printHead(int number);

protected:
	std::vector<State>* mp_states;
	std::vector<Action>* mp_actions;
	std::vector<double>* mp_rewards;
	std::vector<State>* mp_next_states;
	std::vector<bool>* mp_end_of_eps;
	std::vector<double>* mp_td_errors;

};

#endif //STATE_ACTION_MEMORY_H