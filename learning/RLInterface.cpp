#include "RLInterface.h"
#ifdef WIN32
//#include <Windows.h>
#endif
using namespace std;

///////////////Initialization functions///////////////////

RLInterface::RLInterface(void)
{
	//srand(time(NULL));
	//cout << "Creating interface...\n";
	////cout << "\tCreating World ... ";
	////mp_memory = new StateActionMemory(6000);
	//m_reward = 0;
	//cout << "Done.\n";
}

RLInterface::~RLInterface(void)
{
	cout << "Destroying RLInterface::... Goodbye cruel world!" << endl;
	//delete mp_log;
	//delete mp_reward_log;
	delete mp_memory;

	delete mp_world;
	//delete mp_algorithm;
	delete mp_experiment;
	delete mp_parameters;

	delete mp_current_state;
	delete mp_prev_state;

	delete[] mp_current_action->continuousAction;
	delete[] mp_prev_action->continuousAction;
	delete mp_current_action; //bij continue acties: apart het double array deleten
	delete mp_prev_action;

	delete[] mp_torcs_action;
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

///////////////Driver functions///////////////////
double* RLInterface::getAction()
{
	//double* torcs_action = mp_world->convertAction(mp_current_action);
	if(mp_current_action->discrete) 
		mp_world->convertDiscreteAction(mp_current_action, mp_torcs_action);
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

void RLInterface::setState(vector<double>* features)
{
	for(size_t idx = 0; idx < features->size(); idx++){
		if (abs(features->at(idx)) < 0.001)
			mp_current_state->continuousState[idx] = 0;
		else
			mp_current_state->continuousState[idx] = features->at(idx);
	}
	mp_world->setState(mp_current_state);
}

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

void RLInterface::setEOE(){
	//If an episode has ended, keep track of this and make sure that the next state-action pair
	//is not updated with info from previous episode
	mp_parameters->endOfEpisode = true;
}

void RLInterface::setFirstTime(bool val){
	//Dirty hack to avoid having to delete and reconstruct the interface in the onRestart() of driver
	mp_parameters->first_time_step = val;
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


/////////////////////////LEARNING FUNCTIONS ///////////////////////////
bool RLInterface::learningUpdateStep()
{
	return learningUpdateStep(false,RLInterface::RANDOM);
}