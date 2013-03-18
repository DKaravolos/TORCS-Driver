#ifndef STATE_ACTION_MEMORY_H_
#define STATE_ACTION_MEMORY_H_
#include <iostream>
#include <iomanip>
#include <sstream>
#include <deque>

#include "../rlcpp/State.h"
#include "../rlcpp/Action.h"
#include "../utilities/Writer.h"
class StateActionMemory
{
public:
	StateActionMemory();
	StateActionMemory(int max_size);
	~StateActionMemory();

	//LI functions
	void storeTuple	(State* state, Action* action, double reward, State* next_state, 
					bool end_of_ep, double td_error, int option = 0);

	void retrieveTupleAt(int idx, State* state, Action* action, double& reward, 
						State* next_state, bool& end_of_ep);
	void retrieveTupleAt(int idx, State* state, Action* action, double& reward, 
						State* next_state, bool& end_of_ep, double& td_error);
	inline int getSize(){ return mp_states->size();}
	void popBack();

	//print functions
	void printTuple(int index);
	void printHead(unsigned int number);
	void writeTuple(Writer* writer, int index);

protected:
	std::deque<State>* mp_states;
	std::deque<Action>* mp_actions;
	std::deque<double>* mp_rewards;
	std::deque<State>* mp_next_states;
	std::deque<bool>* mp_end_of_eps;
	std::deque<double>* mp_td_errors;
	int m_max_size;

};

#endif //STATE_ACTION_MEMORY_H