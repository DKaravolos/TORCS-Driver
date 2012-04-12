#include "LearningInterface.h"

using namespace std;

LearningInterface::LearningInterface(void)
{
	initState();
	dp_world = new TorcsWorld(this);
	dp_algorithm = new Qlearning("..\source\learning\TorcsWorldCfg.txt", dp_world ) ;
}


LearningInterface::~LearningInterface(void)
{
	delete dp_current_state->continuousState;
	delete dp_current_state;
}


void LearningInterface::initState(){
	dp_current_state = new State;
	dp_current_state->continuous = true;
	dp_current_state->continuous = false;
	dp_current_state->stateDimension = 13;
	dp_current_state->continuousState = new double[13];
}


void LearningInterface::setState(vector<double>* features)
{
	for(size_t idx = 0; idx < features->size(); idx++){
		dp_current_state->continuousState[idx] = features->at(idx);
	}
	dp_world->setState(dp_current_state);
}

void LearningInterface::printState()
{
	cout << "Printing dimensions of State through LearningInterface.\n";

	for(size_t idx = 0; idx < dp_current_state->stateDimension; idx++)
	{
		cout << "Dimension " << idx << " : " << dp_current_state->continuousState[idx] << endl;
	}
}

void LearningInterface::computeReward(int new_dist) {
	d_reward = new_dist - d_last_dist;
	d_last_dist = new_dist;
}

vector<double> LearningInterface::getAction()
{
	vector<double> driver_action;
	Action* world_action = dp_world->getAction();
	if (world_action == NULL)
		cout << "ERROR: request for action, while action is empty!\n";
	else
	{
		size_t dim = world_action->actionDimension;
		for(size_t idx = 0; idx < dim; idx++)
		{
			driver_action.push_back(world_action->continuousAction[idx]);
		}
	}
	return driver_action;
}

double* LearningInterface::experimentMainLoop() {
	//gekopieerd uit runExperiment
	State* state;
	Action * actions = new Action[2] ; //For Sarsa, that needs both the present and next action.

    double reward ;

    //* Training *//
    double * results = new double[ d_param.nResults ] ;

    int episode = 0 ;
    int step = 0 ;
    int result = 0 ;
    double rewardSum = 0.0 ;

    dp_world->reset() ;

    dp_world->getState( state ) ; //state is al ergens gedefinieerd.


    explore( state, action ) ;

    d_param.endOfEpisode = true ;

    int storePer ;

    if ( d_param.train ) {
        storePer = d_param.trainStorePer ;
    } else {
        storePer = d_param.testStorePer ;
    }

    for ( step = 0 ; (step < d_param.nSteps) && (episode < d_param.nEpisodes) ; step++ ) {

        reward = dp_world->act( action ) ; ///MISS MOET DIT ERGENS ANDERS VANDAAN KOMEN

        dp_world->getState( nextState ) ;

        explore( nextState, nextAction ) ;

        rewardSum += reward ;

        d_param.endOfEpisode = dp_world->endOfEpisode() ;

        if ( d_param.train ) {

            if ( d_param.algorithmName.compare("Sarsa") == 0 ) {

                actions[0] = *action ;
                actions[1] = *nextAction ;
                dp_algorithm->update( state, actions, reward, nextState,
					d_param.endOfEpisode, d_param.learningRate, d_param.gamma ) ;

            } else {

                dp_algorithm->update( state, action, reward, nextState,
					d_param.endOfEpisode, d_param.learningRate, d_param.gamma ) ;

            }

        }

        copyState( nextState, state ) ;
        copyAction( nextAction, action ) ;

        if ( d_param.endOfEpisode ) {

            episode++ ;

        }

        // Store results :
        bool store = false ;

        if ( d_param.storePerEpisode && ( episode % storePer == 0 ) && d_param.endOfEpisode ) {

            store = true ;

        } else if ( d_param.storePerStep && ( (step + 1) % storePer == 0 ) ) {

            store = true ;

        }

        if ( store ) {

            results[ result ] = rewardSum/storePer ;
            rewardSum = 0.0 ;
            result++ ;

        }

    }

    delete [] actions ;

    return results ;
}

void LearningInterface::explore( State * state, Action * action) {

    if ( !d_param.train ) {

        dp_algorithm->getMaxAction( state, action ) ;

    } else if ( d_param.boltzmann ) {

        dp_algorithm->explore( state, action, d_param.tau, "boltzmann", d_param.endOfEpisode ) ;

    } else if ( d_param.egreedy ) {

        dp_algorithm->explore( state, action, d_param.epsilon, "egreedy", d_param.endOfEpisode ) ;

    } else if ( d_param.gaussian ) {

        dp_algorithm->explore( state, action, d_param.sigma, "gaussian", d_param.endOfEpisode ) ;

    } else {

        dp_algorithm->getMaxAction( state, action ) ;

    }

}