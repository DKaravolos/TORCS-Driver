# include "Experiment.h"

using namespace std;

Experiment::Experiment(Experiment::Configuration config) {
	cout << "\nIn Experiment constructor:\n";

	nExperiments = 1;
	nAlgorithms = 1;
	//THESE VALUES ARE QUITE RANDOM!
	nSteps = 150000;
	nEpisodes = 10;
	nMaxStepsPerEpisode = 50000;
	nResults =100; //?????
	nTrainSteps = 100000;
	nTrainEpisodes = 10;
	nMaxStepsPerTrainEpisode = 50000;  
	nTrainResults = 100;
	trainStorePer = 100;
	nTestSteps = 0;
	nTestEpisodes = 0;
	nMaxStepsPerTestEpisode = 0;
	nTestResults = 0;
	testStorePer = 0;
	//END OF RANDOM

	switch(config) {
		case QLEARNING:
			//stateDimension = 13;
			//actionDimension = int(-1);
			//discreteStates = int(-1);
			//discreteActions = 15;

			endOfEpisode  = false;

			storePerStep = true;
			storePerEpisode = false;

			boltzmann = true;		////IMPORTANT
			egreedy = false;			////IMPORTANT
			gaussian = false;

			algorithmName = "Qlearning";
			algorithms.push_back(algorithmName);

			learningRateDecreaseType = "none";
			nLearningRates = 1;
			learningRate = new double[nLearningRates];
			learningRate[0] = 0.02; ////IMPORTANT

			tau = 0.02;
			epsilon = 0.01;
			sigma = 0.01;
			taus.push_back(tau);
			epsilons.push_back(epsilon);
			sigmas.push_back(sigma);
			
			gamma = 0.99;
        
			train = true;
			cout << "\tBoltzmann: " << boltzmann << "\t(tau: "<< tau << ")\n";
			cout << "\te-Greedy : " << egreedy << "\t(epsilon: "<< epsilon << ")\n";
			cout << "\tLearningRate: " << learningRate[0] << endl;
			break;

		case CACLA:
			//stateDimension = 13; //waarom zou experiment dit moeten weten? Dit staat toch al in World en Algorithm?
			//actionDimension = 2;
			//discreteStates = 0;
			//discreteActions = 0;

			endOfEpisode  = false;

			storePerStep = true;
			storePerEpisode = false;

			boltzmann = false;		////IMPORTANT
			egreedy = false;			////IMPORTANT
			gaussian = true;

			nAlgorithms = 1;
			algorithmName = "Cacla";
			algorithms.push_back(algorithmName);
			
			learningRateDecreaseType = "none";
			nLearningRates = 2;
			learningRate = new double[nLearningRates];
			learningRate[0] = 0.001; ////Dit wordt niet goed uit de parameterfile gelezen
			learningRate[1] = 0.001;

			tau = 0.01;
			epsilon = 0.1;
			sigma = 0.2;
			//sigma = 0;
			taus.push_back(tau);
			epsilons.push_back(epsilon);
			sigmas.push_back(sigma);
			
			gamma = 0.99;
			
			train = true;
			//cout << "\tBoltzmann exploration: " << boltzmann << endl;
			//cout << "\tGaussian : " << gaussian << endl;
			//cout << "\tLearningRate: " << learningRate[0] << endl;
			break;

		case BAS:
			//stateDimension = 13; //waarom zou experiment dit moeten weten? Dit staat toch al in World en Algorithm?
			//actionDimension = 2;
			//discreteStates = 0;
			//discreteActions = 0;

			endOfEpisode  = false;

			storePerStep = true;
			storePerEpisode = false;

			boltzmann = false;		////IMPORTANT
			egreedy = false;			////IMPORTANT
			gaussian = true;
			
			tau = 0.01;
			epsilon = 0.1;
			sigma = 0.2;
			taus.push_back(tau);
			epsilons.push_back(epsilon);
			sigmas.push_back(sigma);

			nAlgorithms = 1;
			algorithmName = "BinaryActionSearch";
			algorithms.push_back(algorithmName);
			
			learningRateDecreaseType = "none";
			nLearningRates = 1;
			learningRate = new double[nLearningRates];
			learningRate[0] = 0.01;
			
			gamma = 0.99;
			
			train = true;
			//cout << "\tBoltzmann exploration: " << boltzmann << endl;
			cout << "\tGaussian : " << gaussian << endl;
			cout << "\tLearningRate: " << learningRate[0] << endl;
			break;


	case QOS:
			//stateDimension = 13;
			//actionDimension = int(-1);
			//discreteStates = int(-1);
			//discreteActions = 5;

			endOfEpisode  = false;

			storePerStep = true;
			storePerEpisode = false;

			boltzmann = true;		////IMPORTANT
			egreedy = false;			////IMPORTANT
			gaussian = false;
			if(boltzmann)
				cout << "Using Boltzmann exploration.\n";
			else if(egreedy)
				cout << "Using epsilon greedy exploration.\n";

			algorithmName = "Qlearning";
			algorithms.push_back(algorithmName);

			learningRateDecreaseType = "none";
			nLearningRates = 1;
			learningRate = new double[nLearningRates];
			learningRate[0] = 0.02; ////IMPORTANT

			tau = 0.02;
			epsilon = 0.001; //previously epsilon was 0.01
			sigma = 0.01;
			taus.push_back(tau);
			epsilons.push_back(epsilon);
			sigmas.push_back(sigma);
			
			gamma = 0.99;
        
			train = true;
			cout << "\tBoltzmann: " << boltzmann << "\t(tau: "<< tau << ")\n";
			cout << "\te-Greedy : " << egreedy << "\t(epsilon: "<< epsilon << ")\n";
			cout << "\tLearningRate: " << learningRate[0] << endl;
			break;

		default:
			cerr << "There is no default configuration for Experiment.";
			cerr << "Please specify which configuration to use: QLEARNING, CACLA or BAS.\n";
			#ifdef WIN32
					char end;
					cin>>end;
			#endif
			exit(-1);
	}
}


bool Experiment::initializeState( State * state, Algorithm * algorithm, World * world ) {

    if ( algorithm->getContinuousStates() && world->getContinuousStates() ) {

        state->stateDimension   = world->getStateDimension() ;
        state->continuousState  = new double[ state->stateDimension ] ;
        state->continuous       = true ;
        state->discrete         = false ;

    } else if ( algorithm->getDiscreteStates() && world->getDiscreteStates() ) {

        state->numberOfStates   = world->getNumberOfStates() ;
		state->continuousState  = NULL;
        state->continuous       = false ;
        state->discrete         = true ;

    } else {

        cout << "World " << world->getName() << " and algorithm " << algorithm->getName() << " are incompatible, because of different state types (discrete vs continuous). Skipping..." << endl ;

        return true ;

    }

    return false ;

}

bool Experiment::initializeAction( Action * action, Algorithm * algorithm, World * world ) {

    if ( algorithm->getContinuousActions() && world->getContinuousActions() ) {

        action->actionDimension     = world->getActionDimension() ;
        action->continuousAction    = new double[ action->actionDimension ] ;
        action->continuous          = true ;
        action->discrete            = false ;

    } else if ( algorithm->getDiscreteActions() && world->getDiscreteActions() ) {

        action->numberOfActions     = world->getNumberOfActions() ;
		action->continuousAction	= NULL;
        action->continuous          = false ;
        action->discrete            = true ;

    } else {

        cout << "World " << world->getName() << " and algorithm " << algorithm->getName() << " are incompatible, because of different action types (discrete vs continuous). Skipping..." << endl ;

        return true ;

    }

    return false ;

}

Experiment::~Experiment() {

}


void Experiment::setAlgorithm( string algorithmName, World * world ) {

    if ( algorithmName.compare("Q") == 0 ) {

        algorithm = new Qlearning( parameterFile, world ) ;

    } else if ( algorithmName.compare("Sarsa") == 0 ) {

        algorithm = new Sarsa( parameterFile, world ) ;

    } else if ( algorithmName.compare("QV") == 0 ) {

        algorithm = new QVlearning( parameterFile, world ) ;

    } else if ( algorithmName.compare("Acla") == 0 ) {

        algorithm = new Acla( parameterFile, world ) ;

    } else if ( algorithmName.compare("Cacla") == 0 ) {

        algorithm = new Cacla( parameterFile, world ) ;

    } else {

        cout << "Unknown algorithm: " << algorithmName << endl ;
		#ifdef WIN32
			char end;
			cin>>end;
		#endif
        exit(-1) ;

    }

}

void Experiment::getParametersAndRunExperiments( World * world ) {
    int nTau = taus.size() ;
    int nEpsilon = epsilons.size() ;
    int nSigma = sigmas.size() ;

    int nAlgorithms = algorithms.size() ;

    int nExploration = nTau + nEpsilon + nSigma ;

    int nTotalLearningRates = 1 ;

    for ( unsigned int l = 0 ; l < learningRates.size() ; l++ ){

        nTotalLearningRates *= learningRates[l].size() ;

    }

    int nTotalExperiments = nExperiments*nAlgorithms*nExploration*nTotalLearningRates*gammas.size() ;

    cout << "nTotalExperiments " << nTotalExperiments << endl ;

    bool printedWarning1 = false ;

    for ( int i = 0 ; i < nTotalExperiments ; i++ ) {

        bool skip = false ;

        int N = i ;

        algorithmName     = algorithms[ N % nAlgorithms ] ;
        N       /= nAlgorithms ;

        setAlgorithm( algorithmName, world ) ;

        State s1, s2 ;
        Action a1, a2 ;

        state       = &s1 ;
        nextState   = &s2 ;
        action      = &a1 ;
        nextAction  = &a2 ;

        bool f1 = initializeState( state, algorithm, world ) ;
        bool f2 = initializeState( nextState, algorithm, world ) ;
        bool f3 = initializeAction( action, algorithm, world ) ;
        bool f4 = initializeAction( nextAction, algorithm, world ) ;

        skip = f1 || f2 || f3 || f4 ;

        unsigned int nAlgorithmLearningRates = algorithm->getNumberOfLearningRates() ;

        learningRate = new double[ nAlgorithmLearningRates ] ;

        for ( unsigned int l = 0 ; l < learningRates.size() ; l++ ) {

            unsigned int L = learningRates[l].size() ;

            if ( l < nAlgorithmLearningRates ) {

                learningRate[l] = learningRates[l][ N % L ] ;

            } else if ( N % L > 0 ) {
                //if we have more different learning rates than used (i.e. a second
                //or third learning rate for an algorithm that only uses one, such as Sarsa)
                //we skip all redundant experiments.

                skip = true ;

            }

            N  /= L ;

        }

        for ( unsigned int l = learningRates.size() ; l < nAlgorithmLearningRates ; l++ ) {
            //In case that there are less learning rates specified than needed
            //for the current algorithm, reuse the first learning rate for all
            //missing values.

            if ( !printedWarning1 ) {
                cout << "Warning: underspecified number of learning rates for algorithm " << algorithmName << ". " ;
                cout << "(" << learningRates.size() << " learning rate(s) specified, " << nAlgorithmLearningRates << " needed.)" << endl ;
                cout << "Using first specified learning rate for missing values." << endl ;
                printedWarning1 = true ;
            }

            learningRate[l] = learningRate[0] ;

        }

        int exp = N % nExploration ;

        if ( exp < nTau ) {

            boltzmann   = true ;
            tau         = taus[ N % nTau ] ;

            egreedy     = false ;
            epsilon     = 0.0 ;

            gaussian    = false ;
            sigma       = 0.0 ;


        } else if ( exp < (nTau + nEpsilon) ) {

            boltzmann   = false ;
            tau         = 0.0 ;

            egreedy     = true;
            epsilon     = epsilons[ ( N - nTau ) % nEpsilon ] ;

            gaussian    = false ;
            sigma       = 0.0 ;

        } else {

            boltzmann   = false ;
            tau         = 0.0 ;

            egreedy     = false ;
            epsilon     = 0.0 ;

            gaussian    = true ;
            sigma       = sigmas[ ( N - nTau - nEpsilon ) % nSigma ] ;

        }

        if ( algorithmName.compare("Cacla") == 0 ) {

            if ( boltzmann ) {
                //~ cout << "Boltzmann exploration is undefined for Cacla. Skipping..." << endl ;
                skip = true ;
            }

        } else {

            if ( gaussian ) {
                //~ cout << "Gaussian exploration is as of yet undefined for " << algorithmName << ". Skipping..." << endl ;
                skip = true ;
            }
        }

        if ( !skip ) {
            //At this point only skips when:
            // 1) considering learning rates that aren't used by
            //    the current algorithm.
            // 2) initialization of actions and/or states failed.
            //    This happens when the MDP only allows discrete
            //    or continuous spaces and the algorithm vice-
            //    versa.
            //Later may add skipping in other cases, such as when
            //an incompatible exploration scheme is used (such as
            //boltzmann for Cacla).

            N       /= nExploration ;

            gamma   = gammas[ N % gammas.size() ] ;
            N       /= gammas.size() ;

            ostringstream os ;

            os << parameterPath << "_d/" ;

            os << algorithmName << '_' ;
            if ( boltzmann ) {
                os << "B" << tau << '_' ;
            } else if ( egreedy ) {
                os << "E" << epsilon << '_' ;
            } else if ( gaussian ) {
                os << "G" << sigma << '_' ;
            }
            os << 'L' ;
            for ( unsigned int l = 0 ; l < nAlgorithmLearningRates ; l++ ) {
                os << learningRate[l] << '_' ;
            }
            os << 'D' ;
            os << gamma  ;

            cout << "Storing in :  " << os.str() << endl ;

            train               = true ;
            nSteps              = nTrainSteps ;
            nEpisodes           = nTrainEpisodes ;
            nMaxStepsPerEpisode = nMaxStepsPerTrainEpisode ;
            nResults            = nTrainResults ;

            double * results    = runExperiment( world ) ;

            ostringstream ostemp ("") ;
            ostemp << os.str().c_str() << ".onresults" ;
            storeRewards( (ostemp.str()).c_str(), results, nTrainResults ) ;

            cout << "Final train results: " << results[ nTrainResults - 1 ] << endl ;
            delete [] results ;

            train               = false ;
            nSteps              = nTestSteps ;
            nEpisodes           = nTestEpisodes ;
            nMaxStepsPerEpisode = nMaxStepsPerTestEpisode ;
            nResults            = nTestResults ;

            results             = runExperiment( world ) ;

            ostemp.str("") ;
            ostemp << os.str().c_str() << ".offresults" ;
            storeRewards( (ostemp.str()).c_str(), results, nTestResults ) ;

            cout << "Final test results:  " << results[ nTestResults - 1 ] << endl ;
            delete [] results ;

        }

        delete [] learningRate;

        if ( state->continuous ) {
            delete [] state->continuousState ;
            delete [] nextState->continuousState ;
        }

        if ( action->continuous ) {
            delete [] action->continuousAction ;
            delete [] nextAction->continuousAction ;
        }

        delete algorithm ;

    }

    //delete world ;

}

void Experiment::storeRewards( const char * filepath, double * rewards, int nRewards ) {

    ifstream ifile ;

    ifile.open( filepath, ifstream::in ) ;

    int N = 0 ;

    if (ifile) {
        ifile >> N ;
    }

    ofstream ofile ;

    ofile.open( filepath, ofstream::out ) ;
    ofile << (N+1)  << '\n' ;

    double last, lastvar, lastdif ;

    for ( int e = 0 ; e < nRewards ; e++ ) {


        if ( N > 0 ) {

            ifile >> last ;
            ifile >> lastvar ;

            lastdif = (rewards[e] - last) ;
            lastvar = N*lastvar/(N+1.0) + lastdif*lastdif/N ;

            last    = (N*last + rewards[e])/(N+1.0) ;

        } else {

            lastvar = 0 ;
            last    = rewards[e] ;

        }

        ofile << last << " " << lastvar << '\n' ;

    }

    ifile.close() ;

    ofile.close() ;
}

double * Experiment::runExperiment( World * world ) {

    Action * actions = new Action[2] ; //For Sarsa, that needs both the present and next action.

    double reward ;

    //* Training *//
    double * results = new double[ nResults ] ;

    int episode = 0 ;
    int step = 0 ;
    int result = 0 ;
    double rewardSum = 0.0 ;

    world->reset() ;

    world->getState( state ) ; //state is al ergens gedefinieerd.

    explore( state, action ) ;

    endOfEpisode = true ;

    int storePer ;

    if ( train ) {
        storePer = trainStorePer ;
    } else {
        storePer = testStorePer ;
    }

    for ( step = 0 ; (step < nSteps) && (episode < nEpisodes) ; step++ ) {

        reward = world->act( action ) ;

        world->getState( nextState ) ;

        explore( nextState, nextAction ) ;

        rewardSum += reward ;

        endOfEpisode = world->endOfEpisode() ;

        if ( train ) {

            if ( algorithmName.compare("Sarsa") == 0 ) {

                actions[0] = *action ;
                actions[1] = *nextAction ;
                algorithm->update( state, actions, reward, nextState, endOfEpisode, learningRate, gamma ) ;

            } else {

                algorithm->update( state, action, reward, nextState, endOfEpisode, learningRate, gamma ) ;

            }

        }

        copyState( nextState, state ) ;
        copyAction( nextAction, action ) ;

        if ( endOfEpisode ) {

            episode++ ;

        }

        // Store results :
        bool store = false ;

        if ( storePerEpisode && ( episode % storePer == 0 ) && endOfEpisode ) {

            store = true ;

        } else if ( storePerStep && ( (step + 1) % storePer == 0 ) ) {

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

void Experiment::explore( State * state, Action * action ) {

    if ( !train ) {

        algorithm->getMaxAction( state, action ) ;

    } else if ( boltzmann ) {

        algorithm->explore( state, action, tau, "boltzmann", endOfEpisode ) ;

    } else if ( egreedy ) {

        algorithm->explore( state, action, epsilon, "egreedy", endOfEpisode ) ;

    } else if ( gaussian ) {

        algorithm->explore( state, action, sigma, "gaussian", endOfEpisode ) ;

    } else {

        algorithm->getMaxAction( state, action ) ;

    }

}

void Experiment::read_moveTo( ifstream * ifile, string label ) {
    string temp ;

    while ( temp.compare( label ) != 0 && ifile ) {

        *ifile >> temp ;

        if ( ifile->eof() ) {
            cout << "Read error: Could not find label '" << label << "' while reading parameter file '" << parameterFile << "'" << endl ;
            #ifdef WIN32
				char end;
				cin>>end;
			#endif
			exit(-1) ;
        }

    }
}

vector< double > Experiment::read_doubleArray( string temp ) {

    vector< double > parameters ;

    istringstream iss( temp ) ;

    double parameter ;

    while( iss >> parameter ) {

        parameters.push_back( parameter ) ;

    }

    return parameters ;

}

void Experiment::readParameterFile() {

    ifstream ifile ;

    ifile.open( parameterFile, ifstream::in ) ;

    string temp ;

    read_moveTo( &ifile, "nExperiments" ) ;
    ifile >> nExperiments ;

    read_moveTo( &ifile, "steps" ) ;

    read_moveTo( &ifile, "nTrainSteps" ) ;
    ifile >> nTrainSteps ;

    read_moveTo( &ifile, "trainStorePer" ) ;
    ifile >> trainStorePer ;

    read_moveTo( &ifile, "nTestSteps" ) ;
    ifile >> nTestSteps ;

    read_moveTo( &ifile, "testStorePer" ) ;
    ifile >> testStorePer ;

    read_moveTo( &ifile, "nTrainEpisodes" ) ;
    ifile >> nTrainEpisodes ;

    read_moveTo( &ifile, "nTestEpisodes" ) ;
    ifile >> nTestEpisodes ;

    read_moveTo( &ifile, "nMaxStepsPerTrainEpisode" ) ;
    ifile >> nMaxStepsPerTrainEpisode ;

    read_moveTo( &ifile, "nMaxStepsPerTestEpisode" ) ;
    ifile >> nMaxStepsPerTestEpisode ;


    if ( nTrainSteps == 0 ) {

        nTrainSteps     = nTrainEpisodes*nMaxStepsPerTrainEpisode ;
        nTrainResults   = nTrainSteps/trainStorePer ;
        storePerStep    = false ;
        storePerEpisode = true ;

    } else if ( nMaxStepsPerTrainEpisode == 0 ) {

        nTrainEpisodes  = nTrainSteps ;
        nTrainResults   = nTrainSteps/trainStorePer ;
        storePerStep    = true ;
        storePerEpisode = false ;

    }

    if ( nTestSteps == 0 ) {

        nTestSteps     = nTestEpisodes*nMaxStepsPerTestEpisode ;
        nTestResults   = nTestSteps/testStorePer ;
        storePerStep    = false ;
        storePerEpisode = true ;

    } else if ( nMaxStepsPerTestEpisode == 0 ) {

        nTestEpisodes  = nTestSteps ;
        nTestResults   = nTestSteps/testStorePer ;
        storePerStep    = true ;
        storePerEpisode = false ;

    }


    read_moveTo( &ifile, "algorithm" ) ;

    read_moveTo( &ifile, "algorithms" ) ;
    getline( ifile, temp ) ;
    istringstream iss( temp ) ;

    while ( iss >> temp ) {
        algorithms.push_back( temp ) ;
    }

    read_moveTo( &ifile, "tau" ) ;
    getline( ifile, temp ) ;
    taus = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "epsilon" ) ;
    getline( ifile, temp ) ;
    epsilons = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "sigma" ) ;
    getline( ifile, temp ) ;
    sigmas = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "learningRates" ) ;

    read_moveTo( &ifile, "decreaseType" ) ;
    ifile >> learningRateDecreaseType ;

    read_moveTo( &ifile, "nLearningRates" ) ;
    ifile >> nLearningRates ;

    for ( int l = 0 ; l < nLearningRates ; l++ ) {

        read_moveTo( &ifile, "learningRate" ) ;
        getline( ifile, temp ) ;
        learningRates.push_back( read_doubleArray( temp ) ) ;

    }

    read_moveTo( &ifile, "discount" ) ;

    read_moveTo( &ifile, "gamma" ) ;
    getline( ifile, temp ) ;
    gammas = read_doubleArray( temp ) ;

}

void Experiment::readParameterFile(string paramFile ) {
	cout << "reading parameter file" << endl;
	cout << "Experiment: Algorithm pointer is: " << algorithm <<endl;
	cout << "Experiment: World pointer is: " << world << endl;

    ifstream ifile ;

    ifile.open( paramFile, ifstream::in ) ;

    string temp ;

    read_moveTo( &ifile, "nExperiments" ) ;
    ifile >> nExperiments ;

    read_moveTo( &ifile, "steps" ) ;

    read_moveTo( &ifile, "nTrainSteps" ) ;
    ifile >> nTrainSteps ;

    read_moveTo( &ifile, "trainStorePer" ) ;
    ifile >> trainStorePer ;

    read_moveTo( &ifile, "nTestSteps" ) ;
    ifile >> nTestSteps ;

    read_moveTo( &ifile, "testStorePer" ) ;
    ifile >> testStorePer ;

    read_moveTo( &ifile, "nTrainEpisodes" ) ;
    ifile >> nTrainEpisodes ;

    read_moveTo( &ifile, "nTestEpisodes" ) ;
    ifile >> nTestEpisodes ;

    read_moveTo( &ifile, "nMaxStepsPerTrainEpisode" ) ;
    ifile >> nMaxStepsPerTrainEpisode ;

    read_moveTo( &ifile, "nMaxStepsPerTestEpisode" ) ;
    ifile >> nMaxStepsPerTestEpisode ;


    if ( nTrainSteps == 0 ) {

        nTrainSteps     = nTrainEpisodes*nMaxStepsPerTrainEpisode ;
        nTrainResults   = nTrainSteps/trainStorePer ;
        storePerStep    = false ;
        storePerEpisode = true ;

    } else if ( nMaxStepsPerTrainEpisode == 0 ) {

        nTrainEpisodes  = nTrainSteps ;
        nTrainResults   = nTrainSteps/trainStorePer ;
        storePerStep    = true ;
        storePerEpisode = false ;

    }

    if ( nTestSteps == 0 ) {

        nTestSteps     = nTestEpisodes*nMaxStepsPerTestEpisode ;
        nTestResults   = nTestSteps/testStorePer ;
        storePerStep    = false ;
        storePerEpisode = true ;

    } else if ( nMaxStepsPerTestEpisode == 0 ) {

        nTestEpisodes  = nTestSteps ;
        nTestResults   = nTestSteps/testStorePer ;
        storePerStep    = true ;
        storePerEpisode = false ;

    }


    read_moveTo( &ifile, "algorithm" ) ;

    read_moveTo( &ifile, "algorithms" ) ;
    getline( ifile, temp ) ;
    istringstream iss( temp ) ;

    while ( iss >> temp ) {
        algorithms.push_back( temp ) ;
    }

    read_moveTo( &ifile, "tau" ) ;
    getline( ifile, temp ) ;
    taus = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "epsilon" ) ;
    getline( ifile, temp ) ;
    epsilons = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "sigma" ) ;
    getline( ifile, temp ) ;
    sigmas = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "learningRates" ) ;

    read_moveTo( &ifile, "decreaseType" ) ;
    ifile >> learningRateDecreaseType ;

    read_moveTo( &ifile, "nLearningRates" ) ;
    ifile >> nLearningRates ;

    for ( int l = 0 ; l < nLearningRates ; l++ ) {

        read_moveTo( &ifile, "learningRate" ) ;
        getline( ifile, temp ) ;
        learningRates.push_back( read_doubleArray( temp ) ) ;

    }

    read_moveTo( &ifile, "discount" ) ;

    read_moveTo( &ifile, "gamma" ) ;
    getline( ifile, temp ) ;
    gammas = read_doubleArray( temp ) ;

}