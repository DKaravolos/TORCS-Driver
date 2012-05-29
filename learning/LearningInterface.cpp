#include "LearningInterface.h"

using namespace std;

///////////////Initialization functions///////////////////

LearningInterface::LearningInterface(void)
{
	srand(time(NULL));
	cout << "Creating interface...\n";
	//cout << "\tCreating World ... ";
	mp_world = new TorcsWorld(TorcsWorld::QLEARNING);
	mp_log = new Writer("log_files/interface_output.txt");
	//mp_reward_log = new Writer("log_files/QL_cumulative_reward.txt");
	mp_log->write("Interface created.");
	mp_memory = new StateActionMemory(5000);
	m_reward = 0;
	cout << "Done.\n";
}

LearningInterface::~LearningInterface(void)
{
	cout << "Destroying LearningInterface... Goodbye cruel world!" << endl;
	delete mp_log;
	//delete mp_reward_log;
	delete mp_memory;

	delete mp_world;
	delete mp_algorithm;
	delete mp_experiment;
	delete mp_parameters;

	delete mp_current_state;
	delete mp_prev_state;

	delete mp_current_action; //bij continue acties: apart het double array deleten
	delete mp_prev_action;

	delete[] mp_torcs_action;
}

void LearningInterface::init()
{
	cout << "Initalizing remainder of interface.\n";
	mp_algorithm = new Qlearning("TorcsWorldCfg2", mp_world) ;

	mp_experiment = new Experiment(Experiment::DEFAULT_Q);
	mp_experiment->algorithm = mp_algorithm;
	mp_experiment->world = mp_world;
	//mp_experiment->readParameterFile("TorcsWorldCfg2");

	initExperimentParam();
	initState();
	initActions();
	cout << "Done.\n";
}

void LearningInterface::init(const char* nn_filename)
{
	cout << "Initalizing remainder of interface.\n";
	mp_algorithm = new Qlearning("TorcsWorldCfg2", mp_world, nn_filename) ;
	mp_experiment = new Experiment(Experiment::DEFAULT_Q);
	mp_experiment->algorithm = mp_algorithm;
	mp_experiment->world = mp_world;
	//mp_experiment->readParameterFile("TorcsWorldCfg2");

	initExperimentParam();
	initState();
	initActions();
	cout << "Done.\n";
}

void LearningInterface::initState(){
	mp_current_state = new State();
	mp_experiment->initializeState(mp_current_state, mp_algorithm, mp_world);
	
	mp_prev_state = new State();
	mp_experiment->initializeState(mp_prev_state, mp_algorithm, mp_world);
}

void LearningInterface::initActions(){
	mp_current_action = new Action();
	mp_experiment->initializeAction(mp_current_action, mp_algorithm, mp_world);
	
	mp_prev_action = new Action();
	mp_experiment->initializeAction(mp_prev_action, mp_algorithm, mp_world);
	
	mp_torcs_action = new double[2];
	//Let op: mp_current_action en mp_prev_action zijn twee aparte stukken geheugen die geüpdate dienen te worden.
	// Bij voorkeur dus niet naar nieuwe dingen verwijzen, maar huidige waarden aanpassen.
}

void LearningInterface::initExperimentParam()
{
	mp_parameters = new ExperimentParameters();
	mp_parameters->episode = 0 ;
    mp_parameters->step = 0 ;
    mp_parameters->result = 0 ;
    mp_parameters->rewardSum = 0.0 ;
	mp_parameters->endOfEpisode = true;
	mp_parameters->train = true;
	mp_parameters->first_time_step = true;

    if ( mp_parameters->train ) {
        mp_parameters->storePer = mp_experiment->trainStorePer ;
    } else {
        mp_parameters->storePer = mp_experiment->testStorePer ;
    }
}

///////////////Driver functions///////////////////
double* LearningInterface::getAction()
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
		cout << "ERROR: request for action, while action is empty!\n";
		return NULL;
	}
	else	
		return mp_torcs_action;
}

void LearningInterface::setRewardPrevAction(double reward)
{
	m_reward = reward;
}

////////////////// STATE FUNCTIONS ///////////////////////

void LearningInterface::setState(vector<double>* features)
{
	for(size_t idx = 0; idx < features->size(); idx++){
		if (abs(features->at(idx)) < 0.001)
			mp_current_state->continuousState[idx] = 0;
		else
			mp_current_state->continuousState[idx] = features->at(idx);
	}
	mp_world->setState(mp_current_state);
}

void LearningInterface::printState()
{
	cout << "Printing dimensions of State through LearningInterface.\n";

	for(int idx = 0; idx < mp_current_state->stateDimension; idx++) {
		cout << "Dimension " << idx << " : " << mp_current_state->continuousState[idx] << endl;
	}
}

void LearningInterface::logState(int timestamp)
{
	stringstream state_log;
	state_log << timestamp << ": Printing dimensions of State through CaclaLearningI.\n";
	mp_log->write(state_log.str());

	for(int idx = 0; idx < mp_current_state->stateDimension; idx++) {
		stringstream state_log2;
		state_log2 << "Dimension " << idx << " : " << mp_current_state->continuousState[idx];
		mp_log->write(state_log2.str());
	}
}


void LearningInterface::logAction(int timestamp)
{
	double* lp_converted_action = new double[2];
	mp_world->convertDiscreteAction(mp_current_action, lp_converted_action);
	stringstream action;
	action	<< timestamp << ": "
			<< "Actor output:\n\t steer: " << lp_converted_action[0] 
			<< "\n\t accel: " << lp_converted_action[1];
	mp_log->write(action.str());
	delete lp_converted_action;
}

/////////////////////////LEARNING FUNCTIONS ///////////////////////////
bool LearningInterface::learningUpdateStep()
{
	return learningUpdateStep(false, LearningInterface::RANDOM);
}

int g_tuples_in_mem = 0;
bool LearningInterface::learningUpdateStep(bool store_tuples, UpdateOption option)
{
	//Check for stop conditions
	if( (mp_parameters->step >= mp_experiment->nSteps) ){
		cout << "Learning experiment is over. experimentMainLoop will not be ran.\n";
		mp_algorithm->writeQNN("RD_first_run_QNN"); //write NN to file if done with learning
		mp_log->write("Writing QNN after stop condition\n", true);
		return true;
	}

	if(mp_parameters->step % 4500 == 0) {
		mp_algorithm->writeQNN("RD_first_run_QNN"); //write NN every 10.000 steps
		mp_log->write("Writing QNN\n");
	}

	mp_experiment->explore( mp_current_state, mp_current_action); //Computes new action based on current state
	//current_action wordt gevuld.
	
	mp_parameters->rewardSum += m_reward ;
	mp_parameters->endOfEpisode = mp_world->endOfEpisode(); //Check if an episode has ended.
	double l_td_error = 0;

	if ( mp_parameters->train)
	{
		if(!mp_parameters->first_time_step)
		{
			if ( mp_experiment->algorithmName.compare("Sarsa") == 0 )
			{
				cerr << "SARSA is not implemented, please use Qlearning. LearningMainloop is now shutting down.\n";
				return true;

			} else if ( mp_experiment->algorithmName.compare("Qlearning") == 0 ) {
				l_td_error = mp_algorithm->updateAndReturnTDError(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
							mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
				
			} else if ( mp_experiment->algorithmName.compare("Cacla") == 0 ) {
				//mp_algorithm->update(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
				//			mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
				//
				cerr << "This is the QDriver, not the CaclaDriver.Quitting.\n";
				return true;
			}
		} else {
			cout << "First time step. Not performing learning because of invalid state values "  << endl;
			mp_parameters->first_time_step = false;
			copyState( mp_current_state, mp_prev_state ) ;
			copyAction( mp_current_action, mp_prev_action ) ;
			return false;
		}
	}

	//Copy current state and action to history
	if(store_tuples && !mp_parameters->first_time_step	//if not first time step
					&& mp_parameters->step % 10			//store a tuple every 10 steps
					//&& mp_memory->getSize() < 5000		// until a 1000 tuples are stored
	){	
		mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
								mp_parameters->endOfEpisode, l_td_error, option);
		//cout << "Tuples in memory: "<< ++g_tuples_in_mem << endl;
	}
	copyState( mp_current_state, mp_prev_state ) ;
	copyAction( mp_current_action, mp_prev_action ) ;

	/*if(mp_memory->getSize() > 990)
		mp_memory->printTuple(mp_memory->getSize()-1);*/

	//Keep track of time
	if ( mp_parameters->endOfEpisode ) {
		mp_parameters->episode++ ;
	}
	mp_parameters->step++;
	if(mp_parameters->step % 1000 == 0) {
		stringstream message;
		message << "Number of steps so far: " << mp_parameters->step;
		mp_log->write(message.str());
		cout << "Number of steps so far: " << mp_parameters->step << endl;
		cout << "Max learning steps: " << mp_experiment->nSteps << endl;
	}
	return false;
}

void LearningInterface::updateWithOldTuple(UpdateOption option)
{
	if(mp_memory->getSize() == 0) {
		//cout << "Can't update with old tuple if there is no memory" << endl;
		return;
	}
	State* lp_state = new State();
	Action* lp_action = new Action();
	double l_reward = 0;
	State* lp_next_state = new State();
	bool l_end_of_ep = false;
	int tuple_idx = 0;
	double l_td_error;

	if(mp_memory->getSize() >= 5)
		mp_memory->printHead(5);

	switch(option)
	{
		case RANDOM:
			//update with random tuple from memory
			tuple_idx = rand() % mp_memory->getSize();
			//cout << "Retrieving tuple "<<tuple_idx <<endl;
			mp_memory->retrieveTupleAt(tuple_idx,lp_state,lp_action,l_reward,lp_next_state, l_end_of_ep);
			break;

		case TD:
			//update with tuple with high TD error
			tuple_idx = mp_memory->getSize()-1;
			mp_memory->retrieveTupleAt(tuple_idx,lp_state,lp_action,l_reward,lp_next_state, l_end_of_ep, l_td_error);
			cout << "updating tuple with td error of "<< l_td_error << endl;
			break;

		default:
			cout << "This update option does not exist!" <<endl;
			throw std::invalid_argument("Given update option does not exist. Please use RANDOM");
	}

	if(lp_state == NULL || lp_action == NULL || lp_next_state == NULL) {
		cout << "Something went wrong during update from memory." << endl;
		throw "Noo! I can't update my network with NULL pointers!";
	} else {
			//cout << "Updating old tuple\n";
			switch(option)
			{
				case RANDOM:
					mp_algorithm->update(lp_state, lp_action, l_reward, lp_next_state,
										l_end_of_ep, mp_experiment->learningRate, mp_experiment->gamma);
					break;
				case TD:
					//Update network with this tuple
					l_td_error = mp_algorithm->updateAndReturnTDError(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
							mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
							
					//since the TD error has changed, it should be removed and inserted again with the new TD error
					mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
								mp_parameters->endOfEpisode, l_td_error, option);
					break;

					//NOTE TO SELF: DE TUPLE MOET NOG VERWIJDERD WORDEN UIT HET GEHEUGEN EN JE MOET OOK NOG EVEN GOED CHECKEN OF
					//OF HET GEHUGEN NU WEL IN DE GOEDE VOLGORDE OPGEBOUWD WORDT. WELLICHT IS DIT HANDIG OM TE DOEN IN HET 
					//STATE ACTION MEMORY PROJECT!
			}
	}
}