#include "BASLearningInterface.h"

using namespace std;

///////////////Initialization functions///////////////////

BASLearningInterface::BASLearningInterface(void)
{
	srand(unsigned int(time(NULL)));
	cout << "Creating interface...\n";
	//cout << "\tCreating World ... ";
	mp_world = new TorcsWorld(TorcsWorld::BAS);
	mp_log = new Writer("log_files/BASinterface_output.txt");
	mp_log->write("Interface created.");
	//mp_memory = new StateActionMemory(5000);
	m_reward = 0;
	
	//Create adaptive cumulative reward file
	string f_name = "log_files/BAS_cumulative_reward";
	ifstream check;
	
	for(int f_nr = 0; f_nr <=20; f_nr++)
	{
		stringstream file_name;
		file_name << f_name << f_nr << ".txt";
		check.open(file_name.str());
		if(!check.is_open())
		{
			mp_reward_log = new Writer(file_name.str());
			break;
		}
		else
			check.close();
	}

	cout << "Done.\n";
}

BASLearningInterface::~BASLearningInterface(void)
{
	cout << "Destroying BASLearningInterface... Goodbye cruel world!" << endl;
	delete mp_log;
	//delete mp_reward_log;
	delete mp_algorithm;
}

void BASLearningInterface::init()
{
	cout << "Initalizing remainder of interface.\n";
	mp_algorithm = new BinaryActionSearch("TorcsWorldCaclaCfg", mp_world) ;
	//mp_algorithm = new BASWithRoots("TorcsWorldCaclaCfg", mp_world);
	mp_experiment = new Experiment(Experiment::BAS);
	mp_experiment->algorithm = mp_algorithm;
	mp_experiment->world = mp_world;
	//mp_experiment->readParameterFile("TorcsWorldCfg2");

	initExperimentParam();
	initState();
	initActions();
	mp_algorithm->init(mp_current_action);
	askExplore();
	askUpdate();
}

void BASLearningInterface::init(const char* nn_filename)
{
	cout << "Initalizing remainder of interface.\n";
	mp_algorithm = new BinaryActionSearch("TorcsWorldCfg2", mp_world, nn_filename) ;
	//mp_algorithm = new BASWithRoots("TorcsWorldCfg2", mp_world, nn_filename);
	mp_experiment = new Experiment(Experiment::BAS);
	mp_experiment->algorithm = mp_algorithm;
	mp_experiment->world = mp_world;
	//mp_experiment->readParameterFile("TorcsWorldCfg2");

	initExperimentParam();
	initState();
	initActions();
	mp_algorithm->init(mp_current_action);
	cout << "Done.\n";
}

void BASLearningInterface::initState(){
	mp_current_state = new State();
	mp_experiment->initializeState(mp_current_state, mp_algorithm, mp_world);
	
	mp_prev_state = new State();
	mp_experiment->initializeState(mp_prev_state, mp_algorithm, mp_world);
}

void BASLearningInterface::initActions(){
	mp_current_action = new Action();
	mp_experiment->initializeAction(mp_current_action, mp_algorithm, mp_world);
	
	mp_prev_action = new Action();
	mp_experiment->initializeAction(mp_prev_action, mp_algorithm, mp_world);
	
	mp_torcs_action = new double[2];
	//Let op: mp_current_action en mp_prev_action zijn twee aparte stukken geheugen die ge�pdate dienen te worden.
	// Bij voorkeur dus niet naar nieuwe dingen verwijzen, maar huidige waarden aanpassen.
}

/////////////////////////LEARNING FUNCTIONS ///////////////////////////

bool BASLearningInterface::learningUpdateStep(bool store_tuples, UpdateOption option)
{
	//Check for stop conditions
	////CHECK NOG OF HET SCHRIJVEN NAAR BESTANDEN GOED GAAT!!!
	
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
	
	double l_td_error;

	if ( mp_parameters->train)
	{
		if(!mp_parameters->first_time_step)
		{
			mp_parameters->rewardSum += m_reward; //Keep track of cumulative reward for statistics
			stringstream rsum;
			rsum << mp_parameters->rewardSum;
			mp_reward_log->write(rsum.str());
			if ( mp_experiment->algorithmName.compare("BinaryActionSearch") == 0 )
			{
				if(m_update)
				{
					if(option == BASLearningInterface::TD)
						/*l_td_error = mp_algorithm->updateAndReturnTDError(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
									mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);*/
						cerr << "TD reupdates is not implemented.\n";
					else {
						mp_algorithm->update(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
									mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma, BinaryActionSearch::ORIGINAL);
						l_td_error = 0;
					}
				}
			} else {
				cerr << "Wrong algorithm selected. Quitting.\n";
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
	if(store_tuples){
		mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
								mp_parameters->endOfEpisode, l_td_error, option);
	}
	copyState( mp_current_state, mp_prev_state ) ;
	copyAction( mp_current_action, mp_prev_action ) ;

	//Keep track of time / episodes
	if ( mp_parameters->endOfEpisode ) {
		mp_parameters->episode++ ;
		mp_parameters->first_time_step = true;
		mp_parameters->endOfEpisode = false;
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

void BASLearningInterface::updateWithOldTuple(UpdateOption option)
{
	if(mp_memory->getSize() == 0) {
		//cout << "Can't update with old tuple if there is no memory" << endl;
		return;
	}
	//Init local variables
	State* lp_state = new State();
	Action* lp_action = new Action();
	double l_reward = 0;
	State* lp_next_state = new State();
	bool l_end_of_ep = false;
	int tuple_idx = 0;
	double l_td_error;

	//if(mp_memory->getSize() >= 5)
	//	mp_memory->printHead(5);
	if(mp_memory->getSize() >= 5) {
		mp_log->write("Before reupdate:");
		mp_memory->writeTuple(mp_log,mp_memory->getSize()-1);
	}

	//Copy tuple to local variables
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
			cout << "This update option does not exist!" <<endl;
			throw std::invalid_argument("Given update option does not exist. Please use RANDOM");
	}

	//Update algorithm with retrieved tuple
	if(lp_state == NULL || lp_action == NULL || lp_next_state == NULL) {
		cout << "Something went wrong during update from memory." << endl;
		throw "Noo! I can't update my network with NULL pointers!";
	} else {
		//switch(option)
		//{
		//	case RANDOM:
		//		mp_algorithm->update(lp_state, lp_action, l_reward, lp_next_state,
		//							l_end_of_ep, mp_experiment->learningRate, mp_experiment->gamma);
		//		break;

		//	case TD:
		//		//Update network with this tuple
		//		l_td_error = mp_algorithm->updateAndReturnTDError(mp_prev_state, mp_prev_action, m_reward, mp_current_state,
		//				mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma);
		//					
		//		//since the TD error has changed, it should be removed and inserted again with the new TD error
		//		mp_memory->popBack();
		//		mp_memory->storeTuple(mp_prev_state, mp_prev_action, m_reward, mp_current_state, 
		//					mp_parameters->endOfEpisode, l_td_error, option);
		//		break;
		//}
	}
	mp_log->write("Done reupdating. LI");
	//Debug log
	//if(mp_memory->getSize() >= 5) {
	//	mp_log->write("After reupdate:");
	//	mp_memory->writeTuple(mp_log,mp_memory->getSize()-1);
	//}
}

void BASLearningInterface::writeNetwork(int identifier, int step)
{
	stringstream QNN_file;
	//QNN_file << "log_files/QLearning_QNN_ep_" << mp_parameters->episode << "_step_" << mp_parameters->step; 
	QNN_file << "log_files/BASDriver_QNN_id_" << identifier << "_step_" << step << ".txt";
	mp_algorithm->writeQNN(QNN_file.str());
	mp_log->write("Writing QNN\n");
}