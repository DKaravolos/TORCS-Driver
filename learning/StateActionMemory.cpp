#include "StateActionMemory.h"

using namespace std;
StateActionMemory::StateActionMemory()
{
	mp_states = new deque<State>();
	mp_actions = new deque<Action>();
	mp_rewards = new deque<double>();
	mp_next_states = new deque<State>();
	mp_end_of_eps = new deque<bool>();
	mp_td_errors = new deque<double>();
	m_max_size = 10000;
}

StateActionMemory::StateActionMemory(int size)
	:m_max_size(size)
{
	mp_states = new deque<State>();
	mp_actions = new deque<Action>();
	mp_rewards = new deque<double>();
	mp_next_states = new deque<State>();
	mp_end_of_eps = new deque<bool>();
	mp_td_errors = new deque<double>();
}

StateActionMemory::~StateActionMemory()
{
	delete mp_states;
	delete mp_actions;
	delete mp_rewards;
	delete mp_next_states;
	delete mp_end_of_eps;
	delete mp_td_errors;
}

void StateActionMemory::storeTuple(State* state, Action* action, double reward, 
									State* next_state, bool end_of_ep, double td_error, int option)
{
	//cout << "Storing Tuple \n";
	if(state == NULL)
		throw std::invalid_argument("Cannot add NULL state to memory!");
	else if(action == NULL)
		throw std::invalid_argument("Cannot add NULL action to memory!");
	else if(next_state == NULL)
		throw std::invalid_argument("Cannot add NULL next_state to memory!");
	else
	{
		//Check if maximum size is reached. If so, delete most unimportant tuple (the first).
		if(mp_states->size() > m_max_size)
		{
			mp_states->pop_front();
			mp_actions->pop_front();
			mp_rewards->pop_front();
			mp_next_states->pop_front();
			mp_end_of_eps->pop_front();
			mp_td_errors->pop_front();
		}

		if( option == 0) //LearningInterface::UpdateOption::RANDOM
		{
			//If storage option is random, just add a new tuple to the front of the deque.
			mp_states->push_back(*state);
			mp_actions->push_back(*action);
			mp_rewards->push_back(reward);
			mp_next_states->push_back(*next_state);
			mp_end_of_eps->push_back(end_of_ep);
			mp_td_errors->push_back(td_error);
			cout << "option 0\n";
			return;
		}
		else if (option == 1) //LearningInterface::UpdateOption::TD
		{
			//If storage option is TD, add a new tuple at the appriopriate place, based on the TD error.
			//
			//cout << "Reupdate using TD option" << endl;
			deque<State>::iterator st_it = mp_states->begin();
			deque<Action>::iterator act_it = mp_actions->begin();
			deque<State>::iterator nxt_st_it = mp_next_states->begin();
			deque<double>::iterator rew_it = mp_rewards->begin();
			deque<bool>::iterator eoe_it = mp_end_of_eps->begin();
			deque<double>::iterator td_it = mp_td_errors->begin();

			//Special case: deques are empty. Just add the new tuple.
			if(mp_td_errors->size() == 0)
			{
				mp_states->push_back(*state);
				mp_actions->push_back(*action);
				mp_rewards->push_back(reward);
				mp_next_states->push_back(*next_state);
				mp_end_of_eps->push_back(end_of_ep);
				mp_td_errors->push_back(td_error);
			} else 
			{
				//Loop through stored tuples with iterators.
				for(td_it = mp_td_errors->begin(); td_it < mp_td_errors->end(); ++td_it){
					//cout << "Comparing: " << td_error << " with " << *td_it << endl << endl;
					//If new TD error is smaller, insert it in front of the checked tuple.
					if(td_error < *td_it) {
						//cout << "Found a place to insert" << endl;
						mp_states->insert(st_it,*state);
						mp_actions->insert(act_it,*action);
						mp_rewards->insert(rew_it,reward);
						mp_next_states->insert(nxt_st_it,*next_state);
						mp_end_of_eps->insert(eoe_it,end_of_ep);
						mp_td_errors->insert(td_it,td_error);
						break; //The iterators are now invalid, a break from the loop is necessary.
					}

					//If the end is reached, this tuple has the largest td error. So, add it to the end.
					else if ((td_it != mp_td_errors->end()) && (td_it + 1 == mp_td_errors->end()))
					{
						//cout << "Arrived at last element. Pushing back." << endl;
						mp_states->push_back(*state);
						mp_actions->push_back(*action);
						mp_rewards->push_back(reward);
						mp_next_states->push_back(*next_state);
						mp_end_of_eps->push_back(end_of_ep);
						mp_td_errors->push_back(td_error);
						break; //Added the break for similarity with previous if-statement.
					}
					
					//If a larger tuple is not found and we have not yet reached the last tuple, increase the other iterators as well.
					else {
						++st_it;
						++act_it;
						++nxt_st_it;
						++rew_it;
						++eoe_it;
					}

				}
			}
			//Here you can do stuff after inserting the tuple, like printing some part of the current SA Memory.
			//printHead(mp_td_errors->size());
		} else {
			cerr << "Please select an implemented option for StateActionMemory storage" << endl;
		}
	}
}

void StateActionMemory::retrieveTupleAt(int index, State* state, Action* action, double& reward, 
										State* next_state, bool& end_of_ep)
{
	//This is making copies of the objects in the vector.
	*state = mp_states->at(index);
	*action = mp_actions->at(index);
	reward = mp_rewards->at(index);
	*next_state = mp_next_states->at(index);
	end_of_ep = mp_end_of_eps->at(index);
}

void StateActionMemory::retrieveTupleAt(int index, State* state, Action* action, double& reward, 
										State* next_state, bool& end_of_ep, double& td_error)
{
	//This is making copies of the objects in the vector.
	*state = mp_states->at(index);
	*action = mp_actions->at(index);
	reward = mp_rewards->at(index);
	*next_state = mp_next_states->at(index);
	end_of_ep = mp_end_of_eps->at(index);
	td_error = mp_td_errors->at(index);
}

void StateActionMemory::popBack()
{
	mp_states->pop_back();
	mp_actions->pop_back();
	mp_rewards->pop_back();
	mp_next_states->pop_back();
	mp_end_of_eps->pop_back();
	mp_td_errors->pop_back();

}

void StateActionMemory::printTuple(int index)
{
	//Actually, there is no need for pointers. We could use objects (more memory on the stack) 
	//or even just write mp_states->at(index),etc. (less readable).
	//The pointers allocate no new memory, they just point to a location in the vector,
	//so there is no need for a delete.
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
		printf("\nnext_state[1] = %.4f", next_state->continuousState[1]);
	else
		cout << "\nnext_state = " << state->discreteState;

	//print end of episode
	cout << "\tend_of_ep: " << mp_end_of_eps->at(index);

	//print td error
	cout<< "\t TD error: " << mp_td_errors->at(index);
	cout << endl;
}

void StateActionMemory::printHead(unsigned int number)
{
	if(number> mp_states->size()) {
		cerr << "Error in printHead: Can't print more tuples than amount stored in memory!\n";
		return;
	}
	for(unsigned int idx = 0; idx < number; idx++)
	{
		printTuple(idx);
	}
}

void StateActionMemory::writeTuple(Writer* writer, int index)
{
	State* state = &mp_states->at(index);
	Action* action = &mp_actions->at(index);
	State* next_state = &mp_next_states->at(index);
	bool cont_s = state->continuous;
	bool cont_a = action->continuous;

	stringstream sout;
	sout << setprecision(6);

	sout << "Tuple "<<index << " : ";
	//print state
	if(cont_s)
		sout << "state[1] = "<< state->continuousState[1];
	else
		sout << "state = " << state->discreteState;
	
	//print action
	if(cont_a)
		sout << "\taction[0] = " << action->continuousAction[0];
	else
		sout << "\taction = " << action->discreteAction;
	
	//print reward
	sout << "\treward: "<< mp_rewards->at(index);
	
	//print next state
	if(cont_s)
		sout << "\nnext_state[1] = " << next_state->continuousState[1];
	else
		sout << "\nnext_state = " << state->discreteState;

	//print end of episode
	sout << "\tend_of_ep: " << mp_end_of_eps->at(index);

	//print td error
	sout<< "\t TD error: " << mp_td_errors->at(index);
	
	writer->write(sout.str());
}