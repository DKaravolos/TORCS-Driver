#include "TCLearningInterface.h"
#include <stdexcept>
//#include <Windows.h>
using namespace std;
//#define INTERFACE_DEBUG true
#define HAS_INFORMATION true


///////////////Initialization functions///////////////////

TCLearningInterface::TCLearningInterface(const string& log_dir)
{
	srand(unsigned(time(NULL)));
	cout << "Creating interface...\n";
	mp_world = new TorcsWorld(TorcsWorld::QLEARNING);
	m_log_dir = log_dir;
	//mp_reward_log = new Writer(m_log_dir + "TC_cumulative_reward.txt");
	//mp_log = new Writer(m_log_dir + "TC_interface_output.txt");
	m_reward = 0;
	m_eta = 0;
	m_symmetry = false;
	cout << "Done.\n";
}

TCLearningInterface::~TCLearningInterface(void)
{
	cout << "Destroying TCLearningInterface... Goodbye cruel world!" << endl;
	//delete mp_log;
	//delete mp_reward_log;
	delete mp_algorithm;
	delete[] mp_torcs_action;

	if(HAS_INFORMATION){
		delete public_car_control;
		delete public_car_state;
	}
}

void TCLearningInterface::init()
{
	mp_algorithm = new TileCodingHM(mp_world, m_log_dir) ;
	cout << "TileCoding constructed.\n";
	_init(false);
}

void TCLearningInterface::init(const bool& automatic_experiment)
{
	mp_algorithm = new TileCodingHM(mp_world, m_log_dir) ;
	cout << "TileCoding constructed.\n";
	_init(automatic_experiment);
}

void TCLearningInterface::init(const bool& automatic_experiment, const char* qtable_filename)
{
	//mp_algorithm = new TileCodingHM (mp_world, m_log_dir, qtable_filename);
	mp_algorithm = new TileCodingHM (mp_world, m_log_dir, qtable_filename);
	//err << "ERROR: No TileCoding object created!\n";
	_init(automatic_experiment);
}

void TCLearningInterface::_init(const bool& automatic_experiment)
{
	cout << "Initalizing remainder of interface.\n";
	mp_experiment = new Experiment(Experiment::TILECODING); //Het is geen gek idee om de instellingen op te slaan in de log.
	mp_experiment->algorithm = mp_algorithm;
	mp_experiment->world = mp_world;

	if(HAS_INFORMATION){
		public_car_control = new CarControl();
		public_car_state = new CarState();
	}

	initExperimentParam();
	initState();
	initActions();

	if(!automatic_experiment)
	{
		askExplore();
		askUpdate();
	} else {
		cout << "Default value of m_explore and m_update are: " << m_explore << " " << m_update << endl;
	}
	cout << "Done.\n";
}

void TCLearningInterface::initState(){
	mp_current_state = new State();
	mp_experiment->initializeState(mp_current_state, mp_algorithm, mp_world);
	mp_current_state->time_step = 0;
	
	mp_prev_state = new State();
	mp_experiment->initializeState(mp_prev_state, mp_algorithm, mp_world);
	mp_prev_state->time_step = 0;
}

void TCLearningInterface::initActions(){
	mp_current_action = new Action();
	mp_experiment->initializeAction(mp_current_action, mp_algorithm, mp_world);
	
	mp_prev_action = new Action();
	mp_experiment->initializeAction(mp_prev_action, mp_algorithm, mp_world);
	
	mp_torcs_action = new double[2];
	//Let op: mp_current_action en mp_prev_action zijn twee aparte stukken geheugen die ge?pdate dienen te worden.
	// Bij voorkeur dus niet naar nieuwe dingen verwijzen, maar huidige waarden aanpassen.
}

/////////////////////////LEARNING FUNCTIONS ///////////////////////////

bool TCLearningInterface::learningUpdateStep(bool store_tuples, UpdateOption option)
{
	//Compute new action based on current state
	//Whether or not exploration is taken into account depends on the user input
	double random_nr = double(rand())/double(RAND_MAX);

	//Binary way of checking whether an informed action is necessary:
	if(HAS_INFORMATION && (random_nr < m_eta)) /// old: !mp_algorithm->isStateKnown(*mp_current_state)
	{

		//cout << "Doing informed action\n";
		doInformedAction(mp_current_action);
	}
	else if(m_explore && (random_nr <= (m_eta + mp_experiment->epsilon)))
	{
		// Using mp_experiment->explore( mp_current_state, mp_current_action); would cause epsilon to act in the leftover probability mass of eta. That is unwanted.
		mp_algorithm->getRandomAction(mp_current_state, mp_current_action);
	}
	else
	{
		mp_algorithm->getMaxAction(mp_current_state, mp_current_action);
//		cout << "Best action: " << mp_current_action->discreteAction << endl;
		//cout << "NOT EXPLORING!!\n";
	}
	//Current_action now has a value

	double l_td_error = 100; //declare td_error, which might be used for sorting tuples later. initialisation is for 'storeTuple',this should not be necessary.
	if (mp_parameters->train)
	{
		if(!mp_parameters->first_time_step)
		{
			//mp_parameters->rewardSum += m_reward; //Keep track of cumulative reward for statistics
			//stringstream rsum;
			//rsum << mp_parameters->rewardSum;
			//mp_reward_log->write(rsum.str());
			if (mp_experiment->algorithmName.compare("TileCoding") == 0 ) {
				if(m_update && option == RLInterface::RANDOM)
					mp_algorithm->update(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
								 mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
				if(m_update && option == RLInterface::TD)
					l_td_error = mp_algorithm->updateAndReturnTDError(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
								 mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
			} else {
				cerr << "Algorithm name not found. Quitting.\n";
				return true;
			}
		} else {
			cout << "tcli_time: " << mp_parameters->step << ". First time step. Not performing learning because of invalid state values "  << endl;
			mp_parameters->first_time_step = false;
			copyState( mp_current_state, mp_prev_state ) ;
			copyAction( mp_current_action, mp_prev_action ) ;
			return false;
		}
	}
	
	//Update Q-table with symmetrical state and action to increase learning speed
	cout << "Doing a Symmetric update\n";
	doSymmetryUpdate();

	//Update Q-table with symmetrical state and action to increase learning speed
	//cout << "Doing a Symmetric update\n";
	if(m_symmetry)
		doSymmetryUpdate();

	//Copy current state and action to history
	if(store_tuples && m_update)
	{	
		mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
								mp_parameters->endOfEpisode, l_td_error, option);
	}
	copyState( mp_current_state, mp_prev_state ) ;
	copyAction( mp_current_action, mp_prev_action );

	//Keep track of time / episodes
	if (mp_parameters->endOfEpisode)
	{
		cout << "I did an endOfEpisode udpate, so I will turn the flag back to false.\n";
		mp_parameters->episode++ ;
		mp_parameters->first_time_step = true;
		mp_parameters->endOfEpisode = false;
	}
	mp_parameters->step++;
	//if(mp_parameters->step % 1000 == 0)
	//	cout << "Number of steps so far: " << mp_parameters->step << endl;
	return false;
}

void TCLearningInterface::updateWithOldTuple(UpdateOption option)
{
	if(mp_memory->getSize() == 0)
	{
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
	//cout << "Start of reupdate\n";
	//if(mp_memory->getSize() >= 5) {
	//	mp_log->write("Before reupdate:");
	//	mp_memory->writeTuple(mp_log,mp_memory->getSize()-1);
	//}

	switch(option)
	{
		case RANDOM:
			//cout << "Retrieving old tuple.\n";
			//update with random tuple from memory
			tuple_idx = rand() % mp_memory->getSize();
			//cout << "Retrieving tuple "<<tuple_idx <<endl;
			mp_memory->retrieveTupleAt(tuple_idx,lp_state,lp_action,l_reward,lp_next_state, l_end_of_ep);
			break;

		case TD:
			//update with tuple with high TD error
			tuple_idx = mp_memory->getSize()-1;
			mp_memory->retrieveTupleAt(tuple_idx,lp_state,lp_action,l_reward,lp_next_state, l_end_of_ep, l_td_error);
			//cout << "updating tuple with TD error of "<< l_td_error << endl;
			break;

		default:
			cout << "This update option does not exist!" <<endl;
			throw std::invalid_argument("Given update option does not exist. Please use RANDOM");
	}

	if(lp_state == NULL || lp_action == NULL || lp_next_state == NULL)
	{
		cout << "Something went wrong during update from memory." << endl;
		throw "Noo! I can't update my network with NULL pointers!";
	} else 
	{
		//cout << "Updating old tuple\n";
		switch(option)
		{ //START OF ERROR?
			case RANDOM:
				mp_algorithm->update(lp_state, lp_action, l_reward, lp_next_state,
									l_end_of_ep, mp_experiment->learningRate, mp_experiment->gamma);
				break;

			case TD:
				//Update network with this tuple
				l_td_error = mp_algorithm->updateAndReturnTDError(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
						mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
							
				//since the TD error has changed, it should be removed and inserted again with the new TD error

				mp_memory->popBack();
				mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
							mp_parameters->endOfEpisode, l_td_error, option);
				break;
		}
	}
	//cout << "Deleting pointers.\n";
	delete lp_state;
	delete lp_action;
	delete lp_next_state;

} //END OF ERROR?



void TCLearningInterface::loadQTable(int identifier, int step)
{
	stringstream QNN_file;
	QNN_file << m_log_dir << "TC_QTable_id_" << identifier << "_step_" << step;
	//TileCodingHM* lp_tilecoding = static_cast<TileCodingHM*>(mp_algorithm);
	//TileCodingHM* lp_tilecoding = static_cast<TileCodingHM*>(mp_algorithm);
	//lp_tilecoding->loadQTable(QNN_file.str());
	mp_algorithm->loadQTable(QNN_file.str());
}

void TCLearningInterface::loadQTable(string filename)
{
	mp_algorithm->loadQTable(filename);
}

//writeNetwork exists only for inheritance (calls writeQTable)
void TCLearningInterface::writeNetwork(int identifier, int step)
{
	//mp_algorithm->writeStateVisits( m_log_dir + "state_visits.txt");
	//mp_algorithm->writeAverageTDError( m_log_dir + "average_td_errors.txt");
	mp_algorithm->writeStateInfo(m_log_dir + "state_info.txt");
	//cout << "NOTE: only writing state visits, not the QTable.\n";
	writeQTable(identifier, step);
}

void TCLearningInterface::writeQTable(int identifier, int step)
{
	stringstream QNN_file;
	QNN_file << m_log_dir << "TC_QTable_id_" << identifier << "_step_" << step << ".txt";
	//TileCodingHM* lp_tilecoding = static_cast<TileCodingHM*>(mp_algorithm);
	TileCodingHM* lp_tilecoding = static_cast<TileCodingHM*>(mp_algorithm);
	lp_tilecoding->writeQTable(QNN_file.str());

	stringstream msg;
	//msg << "time: " << mp_parameters->step << ". Writing QTable\n";
	//mp_log->write(msg.str());
}


void TCLearningInterface::doInformedAction(Action* action)
{
	////Simple check to see if we use the same state for learning and driving
	//if(state.continuousState[0] != public_state_info.getSpeedX())
	//{
	//	cerr << "ERROR! In TileCodingHM: State from LearningInterface and RLDriver are not the same!!" << endl;
	//	cerr << "TCHM. Speed. LI State: " << state.continuousState[0] << ". CarState: " << public_state_info->getSpeedX() << endl;
	//	return;
	//} 

	//get continuous values from heuristic
	float steer = public_car_control->getSteer();
	float accel = public_car_control->getAccel();
	//stringstream ss;
	//cout << "Steer: " << steer << endl;
	//cout << "Accel: " << accel << endl;
	
	//Discretize dimensions to get discrete action for Q learning
	if(steer >= 0.75) { //1L
		cout << "I would have wanted to go left with more than 0.75\n";
		//mp_log->write("I would have wanted to go left with more than 0.75");
	}
	if(steer <= -0.75) { //1L
		cout << "I would have wanted to go right with more than -0.75\n";
		//mp_log->write("I would have wanted to go right with more than -0.75");
	}

	if(steer >= 0.25) { //0.5L
		if(accel >= 0.33)				//1
			action->discreteAction = 0;
		else if(accel >= -0.33)			//0
			action->discreteAction = 1;
		else							//-1
			action->discreteAction = 2;

	} else if(steer >= 0.02) { //0.1L
		if(accel >= 0.33)				//1
			action->discreteAction = 3;
		else if(accel >= -0.33)			//0
			action->discreteAction = 4;
		else							//-1
			action->discreteAction = 5;

	}else if(steer >= -0.02) { //0.0
		if(accel >= 0.33)				//1
			action->discreteAction = 6;
		else if(accel >= -0.33)			//0
			action->discreteAction = 7;
		else							//-1
			action->discreteAction = 8;

	}else if(steer >= -0.25) { //-0.1R
		if(accel >= 0.33)				//1
			action->discreteAction = 9;
		else if(accel >= -0.33)			//0
			action->discreteAction = 10;
		else							//-1
			action->discreteAction = 11;
	}else { //-0.5R
		if(accel >= 0.33)				//1
			action->discreteAction = 12;
		else if(accel >= -0.33)			//0
			action->discreteAction = 13;
		else							//-1
			action->discreteAction = 14;
	}

	//LELIJKE HACK
	//Speedcap! if speed > 120 either accel = neutral or brake
	if(public_car_state->getSpeedX() >=120 && 
	//	action->discreteAction % 3 == 0)
		(action->discreteAction % 3 == 0 ||
		action->discreteAction % 3 == 1))
		action->discreteAction++;

	//ss << "Discrete action: " << action->discreteAction;
	//cout << "Doing a heuristic action: " << action->discreteAction << endl;

	//mp_log->write(ss.str());
}

void TCLearningInterface::doSymmetryUpdate()
{
	State* lp_sym_prev_state = createSymState(mp_prev_state);
	State* lp_sym_curr_state = createSymState(mp_current_state);
	Action* lp_sym_action = createSymAction(mp_prev_action);

	mp_algorithm->update(lp_sym_prev_state, lp_sym_action, m_reward, lp_sym_curr_state,
						 mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);

	//remove heap memory
	delete[] lp_sym_prev_state->continuousState;
	delete lp_sym_prev_state;
	delete[] lp_sym_curr_state->continuousState;
	delete lp_sym_curr_state;
	delete lp_sym_action;
}