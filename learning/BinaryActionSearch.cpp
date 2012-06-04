#include "BinaryActionSearch.h"
#ifdef WIN32
	#include <time.h>
#endif

BinaryActionSearch::BinaryActionSearch( const char * parameterFile, World * w )
{
	mp_BAS_log = new Writer("log_files/BAS_log.txt");

	//Init variables
	discreteStates = false ;
    continuousStates = true ;
    discreteActions = false ;
    continuousActions = true ;
    actionDimension     = w->getActionDimension() ;
	storedGauss = false ;
	mp_resolution =  new double[actionDimension];
	for(int act = 0; act < actionDimension; act++)
		mp_resolution[act] = 8;

	//Create random seed
    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

	//Read NN parameters from file
	readParameterFile( parameterFile ) ;

	if ( discreteStates ) {
		cerr << "Discrete states not implemented. Quitting.";
		#ifdef WIN32
			char end;
			cin >> end;
		#endif
		exit(-1);
    } else {
		//Create Neural Network
        stateDimension = w->getStateDimension() ;
        int layerSizesA[] = { stateDimension, nHiddenQ, actionDimension } ; //hier kun je extra laag toevoegen
		int QNN_settings[] = {0,1,1};
		
		QNN = new cNeuralNetwork( 1, layerSizesA );
        //for ( int a = 0 ; a < numberOfActions ; a++ ) {
        //    QNN.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ; ///hier kan 1 ->2 voor extra laag
        //}

		//Qs = new double[ numberOfActions ] ;
		//cout << "QL: numberOfActions :" << numberOfActions << endl; 
    }
	//QTarget = new double[ 1 ] ;
}


BinaryActionSearch::BinaryActionSearch(const char * parameterFile, World * w, string QNN_file)
{
	mp_BAS_log = new Writer("log_files/BAS_log.txt");

	//Init variables
	discreteStates      = w->getDiscreteStates() ;
    actionDimension     = w->getActionDimension() ;
	storedGauss = false ;

	//Create random seed
    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

	//Read NN parameters from file
	readParameterFile( parameterFile ) ;

	if ( discreteStates ) {
		cerr << "Discrete states not implemented. Quitting.";
		#ifdef WIN32
			char end;
			cin >> end;
		#endif
		exit(-1);
    } else {
		//Create Neural Network
        stateDimension = w->getStateDimension() ;
		//Qs = new double[ numberOfActions ] ;
		cout << "QL: numberOfActions :" << numberOfActions << endl;
		readQNN(QNN_file);
        //QTarget = new double[ 1 ] ;
    }
}


BinaryActionSearch::~BinaryActionSearch(void)
{
	delete mp_BAS_log;
	if ( discreteStates ) {
        //for ( int s = 0 ; s < numberOfStates ; s++ )
        //    delete [] Q[s] ;
        //delete [] Q ;
    } else {
        //for ( int a = 0 ; a < numberOfActions ; a++ )
        //    delete QNN[a];
        //QNN.clear() ;
        //delete [] Qs ;
		delete QNN;
    }
	delete[] mp_min_vals;
	delete[] mp_max_vals;
	delete[] mp_resolution;
    //delete [] QTarget ;
}

void BinaryActionSearch::readParameterFile( const char * parameterFile ) {
    if ( ! discreteStates )
	{
        ifstream ifile ;
        ifile.open( parameterFile) ;
		if(ifile.is_open())
		{
			read_moveTo( &ifile, "nn" ) ;
			read_moveTo( &ifile, "nHiddenQ" ) ;
			ifile >> nHiddenQ ;

			ifile.close() ;
		} else {
			cerr << "Could not open NN file\n";
		}
    }
}

void BinaryActionSearch::init(Action* action)
{
	
	m_nr_actions = action->actionDimension;
	mp_min_vals = new double[m_nr_actions];
	mp_max_vals = new double[m_nr_actions];

	for(int idx = 0; idx < m_nr_actions; idx++)
	{
		stringstream message;
		mp_min_vals[idx] = action->min_val[idx] ;
		mp_max_vals[idx] = action->max_val[idx] ;
		message << "min val action[" << idx << "] = "<<mp_min_vals[idx] <<endl;
		message << "max val action[" << idx << "] = "<<mp_max_vals[idx] <<endl;
		mp_BAS_log->write(message.str());
	}
}

//RL functions
void BinaryActionSearch::getMaxAction( State * state, Action * action )
{
	stringstream message;

	vector<double> output_action;
	if(action->continuous)
	{
		stringstream msg;
		msg << "Starting Binary search at: \n";
		for(int act_idx = 0; act_idx < action->actionDimension; act_idx++) //iterate over action variables
		{	
			msg << mp_max_vals[act_idx] << endl << mp_min_vals[act_idx] << endl;
			output_action.push_back((mp_max_vals[act_idx] + mp_min_vals[act_idx]) / 2); //init each action variable to middle of its range
			msg << "action["<<act_idx<<"] = " << (mp_max_vals[act_idx] + mp_min_vals[act_idx]) / 2 <<endl;
		}
		mp_BAS_log->write(msg.str());

		double delta;
		for(int act_idx = 0; act_idx < action->actionDimension; act_idx++) //iterate over action variables
		{
			delta = 0; // overbodig: delta.push_back(0); //initialize vector delta of length number_of_actions to zeros
			delta = (mp_max_vals[act_idx] - mp_min_vals[act_idx]) * 
					(	pow (2,(mp_resolution[act_idx]-1) ) /
						(pow(2,mp_resolution[act_idx]) - 1)
					); //set the step size Delta for the current action variable
			
			for(int depth = 0; depth< mp_resolution[act_idx]; depth++) //for all resolution bits of this variable
			{
				delta /= 2;
				//halve the step size
				double act_left = output_action[act_idx] - delta;
				double act_right = output_action[act_idx] + delta;
				double debug_val_left = QNN->forwardPropagate(&act_left)[act_idx];
				double debug_val_right = QNN->forwardPropagate(&act_right)[act_idx];
				
				message << "Comparing at depth "<<depth<<". Left: "<<act_left<<" , right: "<<act_right<<".\n";
				message << "val left: " << debug_val_left << ". val right: "<<debug_val_right << endl;

				if (debug_val_left > debug_val_right){ //compare the two children
					//go left
					output_action[act_idx] = act_left;
					message << "Going left\n";
				}else{
					//go right
					output_action[act_idx] = act_right;
					message << "Going right\n";
				}
			}
			//Is het misschien handig om hier hele kleine waarden af te ronden? bijv. output_action[act_idx] <0.001 wordt 0??

			message << "Final action: " <<output_action[act_idx];
			mp_BAS_log->write(message.str());
			//Put output_actions in Action* action
			action->continuousAction[act_idx] = output_action[act_idx];
		}
		//Stuff after both loops

	}
	else
	{
		cerr << "Binary Action Search on discrete actions is not implemented. Quitting program...";
		#ifdef WIN32
		char end;
		cin >> end;
		#endif
		exit(-5);
	}
}

void BinaryActionSearch::getRandomAction( State * state, Action * action )
{
	//Implemented for inheritance compatibility.
	//You're probably not supposed to be using this.
	for ( int a = 0 ; a < actionDimension ; a++ ) {

		#ifdef WIN32
			action->continuousAction[a] = 2.0*double(rand())/RAND_MAX - 1.0 ;
		#else
			action->continuousAction[a] = 2.0*drand48() - 1.0 ;
		#endif
    }
	cerr << "Are you sure you want to output a random action??\n";
}

void BinaryActionSearch::explore( State * state, Action * action, double explorationRate, std::string explorationType, bool endOfEpisode )
{
	if ( explorationType.compare("boltzmann") == 0 ) {

        cerr << "Wrong exploration choice.You're supposed to use Gaussian exploration." << endl ;
		#ifdef WIN32
			char end;
			cin>>end;
		#endif
        exit(-1) ;

    } else if ( explorationType.compare("egreedy") == 0  ) {

        //egreedy( state, action, explorationRate ) ;
		cerr << "Wrong exploration choice.You're supposed to use Gaussian exploration." << endl ;
		#ifdef WIN32
			char end;
			cin>>end;
		#endif
        exit(-1) ;

    } else if ( explorationType.compare("gaussian") == 0  ) {

        gaussian( state, action, explorationRate ) ;

    } else {

        cerr << "Warning, no exploreType: " << explorationType << endl ;
		#ifdef WIN32
			char end;
			cin>>end;
		#endif
        exit(-1) ;

    }
}

void BinaryActionSearch::update( State * state, Action * action, double rt, State * state_, bool endOfEpisode, double * learningRate, double gamma  )
{
	double * at = action->continuousAction ;
	if(state->continuous)
	{
		double * st = state->continuousState ;
        double * st_ = state_->continuousState ; 
		double* QTarget = new double[m_nr_actions];

        if ( endOfEpisode ) {
			for(int act=0; act< action->actionDimension;act++)
			{
				QTarget[act] = rt;
			}
			QNN->backPropagate(st, QTarget, learningRate[0]) ;

        } else {

			double* Qt_ = QNN->forwardPropagate( st_ );
			double* Qt = QNN->forwardPropagate( st );
			for(int act=0; act< action->actionDimension;act++)
			{
				 QTarget[act] = rt + gamma*(Qt_[act]-Qt[act]) ;
			}
			QNN->backPropagate( st, QTarget, learningRate[0] ) ;
        }

		//LOG CRITIC VALUE
		//stringstream value;
		//value	<< "output netwerk: " << Vt
		//		<< "\tTarget: " << VTarget[ 0 ]
		//		<< "\tReward: " << rt
		//		<< "\tGamma: " << gamma
		//		<< "\tVs_" << Vs_
		//		<< "\tVt_after " << Vt_after
		//		<< "\t learningRate[0] " << learningRate[0]
		//		<< "\t learningRate[1] " << learningRate[1];

		//mp_critic_log->write(value.str());
		delete QTarget;
	} else
		cerr << "Do not use discrete states!\n";

}

double BinaryActionSearch::updateAndReturnTDError( State * state, Action * action, double rt, State * state_,
								bool endOfEpisode, double * learningRate, double gamma  )								
{
	double * at = action->continuousAction ;
	double td_error = 0;

	if(state->continuous)
	{
		double * st = state->continuousState ;
        double * st_ = state_->continuousState ; 
		double* QTarget = new double[m_nr_actions];
        if ( endOfEpisode ) {

			for(int act=0; act< action->actionDimension;act++)
			{
				QTarget[act] = rt;
			}
			QNN->backPropagate(st, QTarget, learningRate[0]);

        } else {

			double* Qt_ = QNN->forwardPropagate( st_ );
			double* Qt = QNN->forwardPropagate( st );
			for(int act=0; act< action->actionDimension;act++)
			{
				td_error += Qt_[act]-Qt[act];
				QTarget[act] = rt + gamma*(Qt_[act]-Qt[act]) ;
			}
			QNN->backPropagate( st, QTarget, learningRate[0] ) ;
        }
		//LOG CRITIC VALUE
		//stringstream value;
		//value	<< "output netwerk: " << Vt
		//		<< "\tTarget: " << VTarget[ 0 ]
		//		<< "\tReward: " << rt
		//		<< "\tGamma: " << gamma
		//		<< "\tVs_" << Vs_
		//		<< "\tVt_after " << Vt_after
		//		<< "\t learningRate[0] " << learningRate[0]
		//		<< "\t learningRate[1] " << learningRate[1];

		//mp_critic_log->write(value.str());
	} else
		cerr << "Do not use discrete states!\n";

	return td_error;
}

//Neural Network functions
void BinaryActionSearch::readQNN(std::string QNN_file)
{
	QNN = new cNeuralNetwork(QNN_file);
}

void BinaryActionSearch::writeQNN(std::string QNN_file)
{
		QNN->writeNetwork(QNN_file);
}

//Gaussian exploration functions
double BinaryActionSearch::gaussianRandom() {
    // Generates gaussian (or normal) random numbers, with mean = 0 and
    // std dev = 1. Used for gaussian exploration.

    if ( storedGauss ) {

        storedGauss = false ;

        return g2 ;

    } else {

        double x, y ;

        double z = 1.0 ;

        while ( z >= 1.0 ) {
			#ifdef WIN32
				x = 2.0*double(rand())/RAND_MAX - 1.0;
				y = 2.0*double(rand())/RAND_MAX - 1.0;
			#else
				x = 2.0*drand48() - 1.0;
				y = 2.0*drand48() - 1.0;
			#endif
            z = x * x + y * y;
        }

        z = sqrt( -2.0 * log( z ) / z );

        g1 = x * z;
        g2 = y * z;

        storedGauss = true ;

        return g1 ;

    }

}

void BinaryActionSearch::gaussian( State * state, Action * action, double sigma ) {

    getMaxAction( state, action ) ;

    for ( int a = 0 ; a < actionDimension ; a++ ) {

        action->continuousAction[a] += sigma*gaussianRandom() ;

    }

}
