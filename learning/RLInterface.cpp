#include "RLInterface.h"
#ifdef WIN32
//#include <Windows.h>
#endif
using namespace std;

///////////////Initialization functions///////////////////

RLInterface::RLInterface(void)
{
	m_explore = true;
	m_update = true;
	//srand(time(NULL));
	mp_memory = new StateActionMemory(10000);
}

RLInterface::~RLInterface(void)
{
	cout << "Destroying RLInterface::... Goodbye cruel world!" << endl;
	delete mp_memory;

	delete mp_world;
	//delete mp_algorithm;
	delete mp_experiment;
	delete mp_parameters;

	if(mp_current_state->continuous){ //bij continue states: apart het double array deleten
		delete[] mp_current_state->continuousState;
		delete[] mp_prev_state->continuousState;
	}
	delete mp_current_state;
	delete mp_prev_state;

	if(mp_current_action->continuous){ //bij continue acties: apart het double array deleten
		delete[] mp_current_action->continuousAction;
		delete[] mp_prev_action->continuousAction;
	}
	delete mp_current_action; 
	delete mp_prev_action;

	//delete[] mp_torcs_action;
}

void RLInterface::initExperimentParam()
{
	mp_parameters = new ExperimentParameters();
	mp_parameters->episode = 0 ;
    mp_parameters->step = 0 ;
    mp_parameters->result = 0 ;
    mp_parameters->rewardSum = 0.0 ;
	mp_parameters->endOfEpisode = false;
	mp_parameters->train = true;
	mp_parameters->first_time_step = true;

    if ( mp_parameters->train ) {
        mp_parameters->storePer = mp_experiment->trainStorePer ;
    } else {
        mp_parameters->storePer = mp_experiment->testStorePer ;
    }
}

void RLInterface::askExplore()
{
	cout << "Do you want to explore? (y/n)\n";
	char answer;
	cin >> answer;
	if(answer == 'y')
	{
		cout << "Exploring.\n";
		m_explore = true;
	} else {
		cout << "Not exploring.\n";
		m_explore = false;
	}
}

void RLInterface::askUpdate()
{
	cout << "Do you want to update the network? (y/n)\n";
	char answer;
	cin >> answer;
	if(answer == 'y')
	{
		cout << "Updating.\n";
		m_update = true;
	} else {
		cout << "Not updating.\n";
		m_update = false;
	}
}

///////////////Driver functions///////////////////
double* RLInterface::getAction()
{
	//double* torcs_action = mp_world->convertAction(mp_current_action);
	if(mp_current_action->discrete){
		//cout << "RLInt Q Action: " << mp_current_action->discreteAction; 
		mp_world->convertDiscreteAction(mp_current_action, mp_torcs_action);
	}
	else if(mp_current_action->continuous)
		mp_world->convertContinuousAction(mp_current_action, mp_torcs_action);
	else {
		cerr << "Action must be discrete or continuous!\n";
		return NULL;
	}

	if (mp_torcs_action == NULL)
	{
		cerr << "ERROR: request for action, while action is empty!\n";
		return NULL;
	}
	else	
		return mp_torcs_action;
}

void RLInterface::setRewardPrevAction(double reward)
{
	m_reward = reward;
}

////////////////// STATE FUNCTIONS ///////////////////////

void RLInterface::setState(vector<double>* features, const int& curr_time)
{
	for(size_t idx = 0; idx < features->size(); idx++){
		if (fabs(features->at(idx)) < 0.001)
			mp_current_state->continuousState[idx] = 0;
		else
			mp_current_state->continuousState[idx] = features->at(idx);
	}
	mp_current_state->time_step = curr_time;
	mp_world->setState(mp_current_state);
}

void RLInterface::setEOE(){
	//If an episode has ended, keep track of this and make sure that the next state-action pair
	//is not updated with info from previous episode
	mp_parameters->endOfEpisode = true;
}

void RLInterface::setFirstTime(bool val){
	//Dirty hack to avoid having to delete and reconstruct the interface in the onRestart() of driver
	mp_parameters->first_time_step = val;
}

///////////////////////LOG FUNCTIONS ////////////////////////////

void RLInterface::printState()
{
	cout << "Printing dimensions of State through RLInterface::.";

	for(int idx = 0; idx < mp_current_state->stateDimension; idx++) {
		cout << "Dimension " << idx << " : " << mp_current_state->continuousState[idx] << endl;
	}
}
void RLInterface::logState(int timestamp)
{
	stringstream state_log;
	state_log << timestamp << ": Printing dimensions of State through RLInterface::.\n";
	mp_log->write(state_log.str());

	for(int idx = 0; idx < mp_current_state->stateDimension; idx++) {
		stringstream state_log2;
		state_log2 << "Dimension " << idx << " : " << mp_current_state->continuousState[idx];
		mp_log->write(state_log2.str());
	}
}
void RLInterface::logAction(int timestamp)
{
	if(mp_current_action->continuous)
	{
		stringstream action;
		action	<< timestamp << ": "
				<< "Actor output:\n\t steer: " << mp_current_action->continuousAction[0] 
				<< "\n\t accel: " << mp_current_action->continuousAction[1];
		mp_log->write(action.str());
	} else {
		stringstream action;
		double* lp_converted_action = new double[2];
		mp_world->convertDiscreteAction(mp_current_action, lp_converted_action);
		action	<< timestamp << ": "
			<< "Actor output: "
				<< "Q-action: " <<  mp_current_action->discreteAction
				<< "\n\t steer: " << lp_converted_action[0]
				<< "\n\t accel: " << lp_converted_action[1];
		mp_log->write(action.str());
		delete lp_converted_action;
	}
}

///////////////////////// OTHER ///////////////////////////
bool RLInterface::learningUpdateStep()
{
	return learningUpdateStep(false,RLInterface::RANDOM);
}

void RLInterface::changeLogWriterTo(string& new_file)
{
	if(mp_log != NULL)
		delete mp_log;
	mp_log = new Writer(new_file);
}

void RLInterface::changeRewardWriterTo(string& new_file)
{
	if(mp_reward_log != NULL)
		delete mp_reward_log;
	mp_reward_log = new Writer(new_file);
}

//Create a symmetrical state from a pointer and return it
//Assumes continuous states
State* RLInterface::createSymState(const State* state)
{
	if(state->discrete)
	{
		cout << "No definition for symmetry conversion of a discrete state!";
		return NULL;
	}

	State* l_sym_state = new State();
	l_sym_state->continuous = true;
	l_sym_state->stateDimension = state->stateDimension;

	l_sym_state->continuousState = new double[state->stateDimension]; //Where is this deleted?
	l_sym_state->continuousState[0] = state->continuousState[0]; 	//= speed, which remains the same
	l_sym_state->continuousState[1] = -state->continuousState[1]; //switch trackpos from - to + or from + to -
	l_sym_state->continuousState[2] = -state->continuousState[2];//switch angle from - to + or from + to -

	l_sym_state->continuousState[3] = state->continuousState[7];  //outer left becomes outer right
	l_sym_state->continuousState[4] = state->continuousState[6];  //inner left becomes inner right
	l_sym_state->continuousState[5] = state->continuousState[5];    //stays the same, because it is the 0 degree sensor
	l_sym_state->continuousState[6] = state->continuousState[4]; //inner right becomes inner left
	l_sym_state->continuousState[7] = state->continuousState[3]; //outer right becomes outer left

	return l_sym_state;
}

Action* RLInterface::createSymAction(const Action* action)
{
	if(action->continuous)
	{
		cout << "No definition for symmetry conversion of a continuous action!";
		return NULL;
	}

	Action* l_sym_action = new Action();
	l_sym_action->discrete = true;
	l_sym_action->continuous = false;
	l_sym_action->discreteAction = action->discreteAction;

	if(action->discreteAction >= 0 && action->discreteAction < 3) //action is 0.5L
		l_sym_action->discreteAction += 12;

	if(action->discreteAction >= 3 && action->discreteAction < 6) //action is 0.1L
		l_sym_action->discreteAction += 6;

	//No need to do anything for actions 6, 7 and 8. No steering involved

	if(action->discreteAction >= 9 && action->discreteAction < 12) //action is 0.1R
		l_sym_action->discreteAction -= 6;

	if(action->discreteAction >= 12 && action->discreteAction < 15) //action is 0.5R
		l_sym_action->discreteAction -= 12;

	return l_sym_action;
}
