#include "TCLearningInterface.h"
//#include <Windows.h>
using namespace std;
//#define INTERFACE_DEBUG true

///////////////Initialization functions///////////////////

TCLearningInterface::TCLearningInterface(const string& log_dir)
{
	srand(time(NULL));
	cout << "Creating interface...\n";
	mp_world = new TorcsWorld(TorcsWorld::QLEARNING);
	m_log_dir = log_dir;
	mp_reward_log = new Writer(m_log_dir + "TC_cumulative_reward.txt");
	//mp_log = new Writer(m_log_dir + "]TC_interface_output.txt");
	//mp_log->write("Interface created.");
	mp_memory = new StateActionMemory(10000);
	m_reward = 0;
	cout << "Done.\n";
}

TCLearningInterface::~TCLearningInterface(void)
{
	cout << "Destroying TCLearningInterface... Goodbye cruel world!" << endl;
	//delete mp_log;
	delete mp_reward_log;
	delete mp_algorithm;
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

	initExperimentParam();
	initState();
	initActions();

	bool test = false; //ALLEEN VOOR TEST DRIVER.exe

	if(test) //ALLEEN VOOR TEST DRIVER.exe
	{
		cout << "TCLI: Using test settings!\n";
		m_explore = false;
		m_update = true;
	} else {
		if(!automatic_experiment)
		{
			askExplore();
			askUpdate();
		} else {
			cout << "Default value of m_explore and m_update are: " << m_explore << " " << m_update << endl;
		}
	}
	cout << "Done.\n";
}

void TCLearningInterface::initState(){
	mp_current_state = new State();
	mp_experiment->initializeState(mp_current_state, mp_algorithm, mp_world);
	
	mp_prev_state = new State();
	mp_experiment->initializeState(mp_prev_state, mp_algorithm, mp_world);
}

void TCLearningInterface::initActions(){
	mp_current_action = new Action();
	mp_experiment->initializeAction(mp_current_action, mp_algorithm, mp_world);
	
	mp_prev_action = new Action();
	mp_experiment->initializeAction(mp_prev_action, mp_algorithm, mp_world);
	
	mp_torcs_action = new double[2];
	//Let op: mp_current_action en mp_prev_action zijn twee aparte stukken geheugen die geüpdate dienen te worden.
	// Bij voorkeur dus niet naar nieuwe dingen verwijzen, maar huidige waarden aanpassen.
}

/////////////////////////LEARNING FUNCTIONS ///////////////////////////

bool TCLearningInterface::learningUpdateStep(bool store_tuples, UpdateOption option)
{
	//Check for stop conditions
	//if( (mp_parameters->step >= mp_experiment->nSteps) ){
	//}

	//Compute new action based on current state
	//Whether or not exploration is taken into account depends on the user input
	if(m_explore)
		mp_experiment->explore( mp_current_state, mp_current_action);	
	else
	{
		mp_algorithm->getMaxAction(mp_current_state, mp_current_action);
		cout << "NOT EXPLORING!!\n";
	}
	//Current_action now has a value

	double l_td_error = 100; //declare td_error, which might be used for sorting tuples later. initialisation is for 'storeTuple',this should not be necessary.

	if (mp_parameters->train)
	{
		if(!mp_parameters->first_time_step)
		{
			mp_parameters->rewardSum += m_reward; //Keep track of cumulative reward for statistics
			stringstream rsum;
			rsum << mp_parameters->rewardSum;
			mp_reward_log->write(rsum.str());
			if (mp_experiment->algorithmName.compare("TileCoding") == 0 ) {
				if(m_update && option == RLInterface::UpdateOption::RANDOM)
					mp_algorithm->update(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
								 mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
				if(m_update && option == RLInterface::UpdateOption::TD)
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

	//Copy current state and action to history
	if(store_tuples && m_update){	
		mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
								mp_parameters->endOfEpisode, l_td_error, option);
	}
	copyState( mp_current_state, mp_prev_state ) ;
	copyAction( mp_current_action, mp_prev_action );

	//Keep track of time / episodes
	if (mp_parameters->endOfEpisode) {
		cout << "I did an endOfEpisode udpate, so I will turn the flag back to false.\n";
		mp_parameters->episode++ ;
		mp_parameters->first_time_step = true;
		mp_parameters->endOfEpisode = false;
	}
	mp_parameters->step++;
	if(mp_parameters->step % 1000 == 0) {
		//stringstream message;
		//message << "Number of steps so far: " << mp_parameters->step;
		//mp_log->write(message.str());
		cout << "Number of steps so far: " << mp_parameters->step << endl;
	}
	return false;
}

void TCLearningInterface::updateWithOldTuple(UpdateOption option)
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
	TileCodingHM* lp_tilecoding = static_cast<TileCodingHM*>(mp_algorithm);
	lp_tilecoding->loadQTable(QNN_file.str());
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
	QNN_file << m_log_dir << "TC_QTable_id_" << identifier << "_step_" << step;
	//TileCodingHM* lp_tilecoding = static_cast<TileCodingHM*>(mp_algorithm);
	TileCodingHM* lp_tilecoding = static_cast<TileCodingHM*>(mp_algorithm);
	lp_tilecoding->writeQTable(QNN_file.str());

	stringstream msg;
	//msg << "time: " << mp_parameters->step << ". Writing QTable\n";
	//mp_log->write(msg.str());
}