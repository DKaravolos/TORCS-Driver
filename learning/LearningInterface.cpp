#include "LearningInterface.h"

using namespace std;

///////////////Initialization functions///////////////////

LearningInterface::LearningInterface(void)
{
	cout << "Creating interface...\n";
	//cout << "\tCreating World ... ";
	mp_world = new TorcsWorld();
	mp_writer = new Writer("interface_output.txt");
	cout << "Done.\n";
}


LearningInterface::~LearningInterface(void)
{
	cout << "Destroying LearningInterface... Goodbye cruel world!" << endl;
	delete mp_writer;
	delete mp_world;
	delete mp_algorithm;
	delete mp_experiment;
	delete mp_parameters;

	delete[] mp_current_state->continuousState;
	delete mp_current_state;
	delete[] mp_prev_state->continuousState;
	delete mp_prev_state;

	delete mp_current_action; //bij continue acties: apart het double array deleten
	delete mp_prev_action;

	delete[] mp_torcs_action;
}

void LearningInterface::init()
{
	cout << "Initalizing remainder of interface.\n";
	cout << "Creating Algorithm ... ";
	mp_algorithm = new Qlearning("TorcsWorldCfg2", mp_world ) ;
	//cout << "in LI init2: mp_algorithm is at " << mp_algorithm << endl;
	//cout << "in LI init2: mp_world is at " << mp_world << endl;
	cout << "Done.\n";

	cout << "Creating Experiment ... ";
	mp_experiment = new Experiment(Experiment::DEFAULT_Q);
	cout << "Setting pointers to algorithm and world in experiment\n";
	mp_experiment->algorithm = mp_algorithm;
	mp_experiment->world = mp_world;
	cout << "Done.\n";

	cout << "Reading parameterfile ... ";
	mp_experiment->readParameterFile("TorcsWorldCfg2");
	cout << "Done.\n";

	cout << "Initializing experiment runtime parameters ..." << endl;
	initExperimentParam();
	cout << "Done.\n";

	initState();
	initActions();
}


void LearningInterface::initState(){
	mp_current_state = new State();
	mp_experiment->initializeState(mp_current_state, mp_algorithm, mp_world);
	
	mp_prev_state = new State();
	mp_experiment->initializeState(mp_prev_state, mp_algorithm, mp_world);
	/*mp_current_state->continuous = true;
	mp_current_state->discrete = false;
	mp_current_state->stateDimension = 13;
	mp_current_state->continuousState = new double[13];
	*/
}

void LearningInterface::initActions(){
	mp_current_action = new Action();
	mp_experiment->initializeAction(mp_current_action, mp_algorithm, mp_world);
	
	mp_prev_action = new Action();
	mp_experiment->initializeAction(mp_prev_action, mp_algorithm, mp_world);
	
	mp_torcs_action = new double[2];
	//Let op: mp_current_action en mp_prev_action zijn twee aparte stukken geheugen die ge�pdate dienen te worden.
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
	mp_world->convertAction(mp_current_action, mp_torcs_action);
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


/////////////////////////LEARNING FUNCTIONS ///////////////////////////

bool LearningInterface::learningUpdateStep() {
	//if((mp_parameters->step >= mp_experiment->nSteps) || (mp_parameters->episode >= mp_experiment->nEpisodes)) {
	/*if( (mp_parameters->step >= 1000000) || (mp_parameters->episode >= 100) ){
		cout << "Learning experiment is over. experimentMainLoop will not be ran.\n";
		return true;
	}
	*/
	//commented code with //// is from Experiment
	////reward = world->act( action ) ;
	//not necessary, we already have m_reward

	////world->getState( nextState ) ;
	//not necessary, we already have mp_current_state

	mp_experiment->explore( mp_current_state, mp_current_action); //Computes new action based on current state
	//current_action wordt gevuld.
	
	mp_parameters->rewardSum += m_reward ;
	mp_parameters->endOfEpisode = mp_world->endOfEpisode(); //Weten of episode is gee�ndigd. moet nog ingesteld worden?

	if ( mp_parameters->train)
	{
		if(!mp_parameters->first_time_step)
		{
			if ( mp_experiment->algorithmName.compare("Sarsa") == 0 ) {

				/*actions[0] = *action ;
				actions[1] = *nextAction ;
				algorithm->update( mp_current_state, actions, m_reward, nextState, 
					mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma ) ;
				*/
				cout << "SARSA is not implemented yet, please use Q-learning. LearningMainloop is now shutting down.\n";
				return true;

			} else {
				//Q values are updated with last state, last action and resulting reward and new state
				//cout << "LI update: prev_state: state[0] = "<< mp_prev_state->continuousState[0] << endl;

				mp_algorithm->update( mp_prev_state, mp_prev_action, m_reward, mp_current_state,
					mp_parameters->endOfEpisode, mp_experiment->learningRate, mp_experiment->gamma ) ;

			}
		} else {
			cout << "First time step. Not performing learning because of invalid state values "  << endl;
			mp_parameters->first_time_step = false;
		}
	}

	copyState( mp_current_state, mp_prev_state ) ;
	copyAction( mp_current_action, mp_prev_action ) ;

	if ( mp_parameters->endOfEpisode ) {
		mp_parameters->episode++ ;
	}

	mp_parameters->step++;
	if(mp_parameters->step % 200 == 0) {
		cout << "Number of steps so far: " << mp_parameters->step << endl;
		cout << "Max learning steps: " << mp_experiment->nSteps << endl;
	}
	return false;
}
