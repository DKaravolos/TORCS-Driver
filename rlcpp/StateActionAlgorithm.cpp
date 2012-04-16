# include "StateActionAlgorithm.h"

StateActionAlgorithm::StateActionAlgorithm() {
    continuousStates = true ;
    discreteStates = true ;
    continuousActions = false ;
    discreteActions = true ;
}

void StateActionAlgorithm::getMaxActionFirst( State * state, Action * action ) {

    if ( state->discrete ) {

        Qs = Q[ state->discreteState ] ;

    } else {

        for ( int a = 0 ; a < numberOfActions ; a++ ) {

            Qs[a] = QNN[a]->forwardPropagate( state->continuousState )[0] ;

        }

    }

    action->discreteAction = myArgmax( Qs, numberOfActions ) ;

}

void StateActionAlgorithm::getMaxActionRandom( State * state, Action * action ) {

    if ( state->discrete ) {

        Qs = Q[ state->discreteState ] ;

    } else {

        for ( int a = 0 ; a < numberOfActions ; a++ ) {

            Qs[a] = QNN[a]->forwardPropagate( state->continuousState )[0] ;

        }

    }

    vector<int> maxA = argmaxAll( Qs, numberOfActions ) ;

    int n = maxA.size() ;
	#ifdef WIN32
		action->discreteAction = maxA[ (int) ( n* double(rand())/RAND_MAX ) ] ;
	#else
		action->discreteAction = maxA[ (int) ( n*drand48() ) ] ;
	#endif

}

void StateActionAlgorithm::getMaxAction( State * state, Action * action ) {

    getMaxActionRandom( state, action ) ;

}

void StateActionAlgorithm::getRandomAction( State * state, Action * action ) {

	#ifdef WIN32
		action->discreteAction = (int) ( numberOfActions*double(rand())/RAND_MAX ) ;
	#else
		action->discreteAction = (int) ( numberOfActions*drand48() ) ;
	#endif
}


void StateActionAlgorithm::explore( State * state, Action * action, double explorationRate, string explorationType, bool endOfEpisode ) {

    if ( explorationType.compare("boltzmann") == 0 ) {

        boltzmann( state, action, explorationRate ) ;

    } else if ( explorationType.compare("egreedy") == 0  ) {

        egreedy( state, action, explorationRate ) ;

    } else if ( explorationType.compare("gaussian") == 0  ) {

        cout << "You are trying to use gaussian exploration for an algorithm that" << endl ;
        cout << "does not support it. Please check your parameter file." << endl ;
        exit(0) ;

    } else {

    cout << "Warning, no exploreType: " << explorationType << endl ;
    exit(0) ;

    }

    action->continuous  = false ;
    action->discrete    = true ;

}
void StateActionAlgorithm::egreedy( State * state, Action * action, double epsilon ) {

	#ifdef WIN32
		double random_nr = double(rand())/RAND_MAX;
	#else
		double random_nr = drand48();
	#endif

    if ( random_nr < epsilon ) {

        getMaxAction( state, action ) ;

    } else {

        getRandomAction( state, action ) ;

    }

}

void StateActionAlgorithm::boltzmann( State * state, Action * action, double tau ) {

    setQs( state, action ) ;

    boltzmann( action, tau ) ;

}

void StateActionAlgorithm::setQs( State * state, Action * action ) {

    if ( state->discrete ) {

        Qs = Q[state->discreteState] ;

    } else {

        for ( int a = 0 ; a < numberOfActions ; a++ ) {
            Qs[a] = QNN[a]->forwardPropagate( state->continuousState )[0] ;
        }

    }

}


void StateActionAlgorithm::boltzmann( Action * action, double tau ) {

    double sumQs = 0 ;
    double maxQs = myMax( Qs, numberOfActions ) ;

    for ( int a = 0 ; a < numberOfActions ; a++ ) {
        policy[a]   = exp( (Qs[a]-maxQs)/tau ) ;
        sumQs       += policy[a] ;
    }

    for ( int a = 0 ; a < numberOfActions ; a++ ) {
        policy[a] /= sumQs ;
    }

   	#ifdef WIN32
		double rnd = double(rand())/RAND_MAX;
	#else
		double rnd = drand48();
	#endif


    double total    = policy[0] ;
    int a           = 0 ;

    while ( total < rnd ) {
        a++ ;
        total += policy[ a ] ;
        if ( a >= numberOfActions ) {
            // Something went wrong...
            cout << total << " " << rnd << endl ;
            for ( int _a = 0 ; _a < numberOfActions ; _a++ ) {
                cout << policy[_a] << " " ;
            }
            cout << endl ;
            for ( int _a = 0 ; _a < numberOfActions ; _a++ ) {
                cout << Qs[_a] << " " ;
            }
            cout << endl ;
            exit(0) ;
        }
    }

    action->discreteAction = a ;

}

bool StateActionAlgorithm::getContinuousStates() {
    return continuousStates ;
}

bool StateActionAlgorithm::getDiscreteStates() {
    return discreteStates ;
}

bool StateActionAlgorithm::getContinuousActions() {
    return continuousActions ;
}

bool StateActionAlgorithm::getDiscreteActions() {
    return discreteActions ;
}