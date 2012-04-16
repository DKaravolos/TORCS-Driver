#include "LearningInterface.h"

using namespace std;

LearningInterface::LearningInterface(void)
{
	cout << "\tCreating World ... ";
	mp_world = new TorcsWorld();
	cout << "Done.\n";

	cout << "\tCreating Algorithm ... ";
	mp_algorithm = new Qlearning("CartpoleCfg", mp_world ) ;
	cout << "Done.\n";

	cout << "\tCreating Experiment ... ";
	mp_experiment = new Experiment();
	cout << "Done.\n";

	cout << "\tReading parameterfile ... ";
	mp_experiment->readParameterFile("CartpoleCfg");
	cout << "Done.\n";

	initState();
	initActions();
}


LearningInterface::~LearningInterface(void)
{
	delete[] mp_current_state->continuousState;
	delete mp_current_state;
	delete mp_world;
	delete mp_algorithm;
	delete mp_experiment;

	delete mp_current_action; //bij continue acties: apart het double array deleten
	delete mp_prev_action;
}

///////////////Driver functions///////////////////
double* LearningInterface::getAction()
{
	Action* world_action = mp_world->getAction(); //MOET DEZE ACTION NOG WEL UIT WORLD KOMEN??
	if (world_action == NULL)
	{
		cout << "ERROR: request for action, while action is empty!\n";
		return NULL;
	}
	else
		return world_action->continuousAction;
}

void LearningInterface::setRewardPrevAction(int distance)
{
	m_reward = distance;
}

///////

void LearningInterface::initState(){
	mp_current_state = new State;
	mp_experiment->initializeState(mp_current_state, mp_algorithm, mp_world);
	
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
	
	//Let op: mp_current_action en mp_prev_action zijn twee aparte stukken geheugen die geüpdate dienen te worden.
	// Bij voorkeur dus niet naar nieuwe dingen verwijzen, maar huidige waarden aanpassen.
}


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

	for(int idx = 0; idx < mp_current_state->stateDimension; idx++)
	{
		cout << "Dimension " << idx << " : " << mp_current_state->continuousState[idx] << endl;
	}
}

/*void LearningInterface::computeReward(int new_dist) {
	d_reward = new_dist - d_last_dist;
	d_last_dist = new_dist;
}
*/

double* LearningInterface::experimentMainLoop() {
	//double* pointer = new double(1230.000);


	return pointer;
}
/*
void LearningInterface::explore( State * state, Action * action) {

    if ( !d_param.train ) {

        mp_algorithm->getMaxAction( state, action ) ;

    } else if ( d_param.boltzmann ) {

        mp_algorithm->explore( state, action, d_param.tau, "boltzmann", d_param.endOfEpisode ) ;

    } else if ( d_param.egreedy ) {

        mp_algorithm->explore( state, action, d_param.epsilon, "egreedy", d_param.endOfEpisode ) ;

    } else if ( d_param.gaussian ) {

        mp_algorithm->explore( state, action, d_param.sigma, "gaussian", d_param.endOfEpisode ) ;

    } else {

        mp_algorithm->getMaxAction( state, action ) ;

    }

}*/