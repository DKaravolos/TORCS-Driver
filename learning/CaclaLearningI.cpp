#include "CaclaLearningI.h"
//#include <Windows.h>
using namespace std;

///////////////Initialization functions///////////////////

CaclaLearningI::CaclaLearningI(void)
{
	srand(time(NULL));
	cout << "Creating interface...\n";
	//cout << "\tCreating World ... ";
	mp_world = new TorcsWorld(TorcsWorld::CACLA);
	mp_log = new Writer("log_files/cacla_interface_output.txt");
	mp_reward_log = new Writer("log_files/cacla_cumulative_reward.txt");
	mp_log->write("Interface created.");
	mp_memory = new StateActionMemory(10000);
	m_reward = 0;
	cout << "Done.\n";
}

CaclaLearningI::~CaclaLearningI(void)
{
	cout << "Destroying CaclaLearningI... Goodbye cruel world!" << endl;
	delete mp_log;
	delete mp_reward_log;
	delete mp_algorithm;
}

void CaclaLearningI::init()
{
	mp_algorithm = new Cacla("TorcsWorldCfg20", mp_world) ;
	//cout << "NOTE: USING ONLY 10 HIDDEN NODES!\n"; //normally we use Cfg2, which has 30 nodes
	_init();
}

void CaclaLearningI::init(const char* ann_filename, const char* vnn_filename)
{
	mp_algorithm = new Cacla("TorcsWorldCfg20", mp_world, ann_filename, vnn_filename) ;
	//cout << "NOTE: USING ONLY 10 HIDDEN NODES!\n"; //normally we use Cfg2, which has 30 nodes
	_init();
}

//Added for inheritance reasons
void CaclaLearningI::init(const char* ann_filename)
{
	cerr << "Are you sure you want to initialize Cacla with only one network??\n";
	mp_algorithm = new Cacla("TorcsWorldCfg20", mp_world, ann_filename, ann_filename) ;
	//cout << "NOTE: USING ONLY 10 HIDDEN NODES!\n"; //normally we use Cfg2, which has 30 nodes
	_init();
}

void CaclaLearningI::_init()
{
	cout << "Initalizing remainder of interface.\n";
	mp_experiment = new Experiment(Experiment::CACLA);
	mp_experiment->algorithm = mp_algorithm;
	mp_experiment->world = mp_world;
	//mp_experiment->readParameterFile("TorcsWorldCaclaCfg");

	initExperimentParam();
	initState();
	initActions();
	askExplore();
	askUpdate();
	cout << "Done.\n";
}

void CaclaLearningI::initState(){
	mp_current_state = new State();
	mp_experiment->initializeState(mp_current_state, mp_algorithm, mp_world);
	
	mp_prev_state = new State();
	mp_experiment->initializeState(mp_prev_state, mp_algorithm, mp_world);
}

void CaclaLearningI::initActions(){
	mp_current_action = new Action();
	mp_experiment->initializeAction(mp_current_action, mp_algorithm, mp_world);
	
	mp_prev_action = new Action();
	mp_experiment->initializeAction(mp_prev_action, mp_algorithm, mp_world);
	
	mp_torcs_action = new double[mp_current_action->actionDimension];
	//Let op: mp_current_action en mp_prev_action zijn twee aparte stukken geheugen die geüpdate dienen te worden.
	// Bij voorkeur dus niet naar nieuwe dingen verwijzen, maar huidige waarden aanpassen.
}

/////////////////////////LEARNING FUNCTIONS ///////////////////////////

bool CaclaLearningI::learningUpdateStep(bool store_tuples, UpdateOption option)
{
	//Check for stop conditions
	//is done in Driver now.

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
	double l_td_error; //declare td_error, which might be used for sorting tuples later

	if ( mp_parameters->train)
	{
		if(!mp_parameters->first_time_step)
		{
			mp_parameters->rewardSum += m_reward; //Keep track of cumulative reward for statistics
			stringstream rsum;
			rsum << mp_parameters->rewardSum;
			mp_reward_log->write(rsum.str());
			if (mp_experiment->algorithmName.compare("Cacla") == 0 ) {
				if(m_update && option == UpdateOption::RANDOM)
					mp_algorithm->update(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
								 mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
				if(m_update && option == UpdateOption::TD)
					l_td_error = mp_algorithm->updateAndReturnTDError(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
								 mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
			} else {
				cerr << "Algorithm name not found. Quitting.\n";
				return true;
			}
		} else {
			cout << "li_time: " << mp_parameters->step << ". First time step. Not performing learning because of invalid state values "  << endl;
			mp_current_action->continuousAction[0] = 0;
			mp_current_action->continuousAction[1] = 1; //28-08: Er staat geen uitleg bij. Eerste actie is nu gas geven zonder sturen. Is dat niet valsspelen?
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
	if ( mp_parameters->endOfEpisode ) {
		mp_parameters->episode++ ;
		mp_parameters->first_time_step = true;
		mp_parameters->endOfEpisode = false;
	}
	mp_parameters->step++;
	//if(mp_parameters->step % 1000 == 0) {
	//	stringstream message;
	//	message << "Number of steps so far: " << mp_parameters->step;
	//	mp_log->write(message.str());
	//	cout << "Number of steps so far: " << mp_parameters->step << endl;
	//	//cout << "Max learning steps: " << mp_experiment->nSteps << endl;
	//}

	return false;
}

void CaclaLearningI::updateWithOldTuple(UpdateOption option)
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

	//if(mp_memory->getSize() >= 5) {
	//	mp_log->write("Before reupdate:");
	//	mp_memory->writeTuple(mp_log,mp_memory->getSize()-1);
	//}

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
			//cout << "updating tuple with TD error of "<< l_td_error << endl;
			break;

		default:
			cerr << "This update option does not exist!" <<endl;
			throw std::invalid_argument("Given update option does not exist. Please use RANDOM");
	}

	if(lp_state == NULL || lp_action == NULL || lp_next_state == NULL) {
		cerr << "Something went wrong during update from memory." << endl;
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

					mp_memory->popBack();
					mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
								mp_parameters->endOfEpisode, l_td_error, option);
					break;
			}
	}
	//if(mp_memory->getSize() >= 5) {
	//	mp_log->write("After reupdate:");
	//	mp_memory->writeTuple(mp_log,mp_memory->getSize()-1);
	//}
}

void CaclaLearningI::writeNetwork(int identifier, int step)
{
		stringstream ANN_file;
		stringstream VNN_file;
		ANN_file << "log_files/Cacla_ANN_id_" << identifier << "_step_"<< step;
		VNN_file << "log_files/Cacla_VNN_id_" << identifier << "_step_" << step;
		mp_algorithm->writeNN(ANN_file.str(), VNN_file.str());
		mp_log->write("Writing ANN and VNN\n");
}