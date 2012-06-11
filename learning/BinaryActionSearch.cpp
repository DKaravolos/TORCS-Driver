#include "BinaryActionSearch.h"
#ifdef WIN32
	#include <time.h>
#endif

BinaryActionSearch::BinaryActionSearch( const char * parameterFile, World * w )
{
	_init(parameterFile,w);
	if ( discreteStates )
	{
		cerr << "Discrete states not implemented. Quitting.";
		#ifdef WIN32
			char end;
			cin >> end;
		#endif
		exit(-1);
    } else
	{
		//Create Neural Network
        stateDimension = w->getStateDimension() ;
        int layerSizesA[] = { stateDimension + actionDimension, nHiddenQ, 1 } ; //state dimension+1, omdat de actie erbij komt. hier kan extra laag
		int QNN_settings[] = {0,1,1};
		
		//QNN = new cNeuralNetwork( 1, layerSizesA );
        for ( int a = 0 ; a < actionDimension ; a++ ) {
            QNN.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ; ///hier kan 1 ->2 voor extra laag
        }
    }
}


BinaryActionSearch::BinaryActionSearch(const char * parameterFile, World * w, string QNN_file)
{
	_init(parameterFile,w);
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
		cout << "QL: numberOfActions :" << actionDimension << endl;
		readQNN(QNN_file);
    }
}


void BinaryActionSearch::_init(const char* parameterFile, World* w)
{
	mp_BAS_log = new Writer("log_files/BAS_log.txt");

	//Init variables
	//discreteStates = false ;//(is dit nog wel nodig??
	//continuousStates = true ;//(is dit nog wel nodig??
	//discreteActions = false ;//(is dit nog wel nodig??
	//continuousActions = true ;//(is dit nog wel nodig??
    actionDimension     = w->getActionDimension() ;
	storedGauss = false ;
	mp_resolution =  new double[actionDimension];
	//Set the search resolution per action dimension
	for(int act = 0; act < actionDimension; act++)
		mp_resolution[act] = 5;

	//Create random seed
    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

	//Read NN parameters from file
	readParameterFile( parameterFile ) ;
}


BinaryActionSearch::~BinaryActionSearch(void)
{
	delete mp_BAS_log;
	if ( discreteStates ) {
        //for ( int s = 0 ; s < numberOfStates ; s++ )
        //    delete [] Q[s] ;
        //delete [] Q ;
    } else {
        for ( int a = 0 ; a < numberOfActions ; a++ )
            delete QNN[a];
        QNN.clear() ;
    }
	delete[] mp_min_vals;
	delete[] mp_max_vals;
	delete[] mp_resolution;
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

	if(action->continuous)
	{
		//Create input for neural network
		int l_inputsize = state->stateDimension + action->actionDimension;
		int l_state_dim = state->stateDimension; //this is used as index for the NN_input
		int l_action_dim = action->actionDimension;
		message << "inputsize: " << l_inputsize << endl;
		message << "l_state_dim: " << l_state_dim << endl;
		message << "l_action_dim: " << l_action_dim << endl;

		vector<double>* NN_input = new vector<double>;
		for(int idx = 0; idx < state->stateDimension; idx++)
		{
			NN_input->push_back(state->continuousState[idx]);
		}

		//Init action variables for binary search
		stringstream extra_msg;
		message << "Starting Binary search at: \n";
		for(int act_idx = 0; act_idx < l_action_dim; act_idx++) //iterate over action variables
		{	
			double l_mid_range = (mp_max_vals[act_idx] + mp_min_vals[act_idx]) / 2;
			NN_input->push_back(l_mid_range); //init each action variable to middle of its range
			//message << "midrange of action["<<act_idx<<"] = " << l_mid_range <<endl;
		}

		for(int idx = 0; idx < l_inputsize; idx++)
		{
			message<< "NN input " << idx << " : "<< NN_input->at(idx) << endl;
		}

		//Do the actual search
		double delta;
		for(int act_idx = 0; act_idx < l_action_dim; act_idx++) //iterate over action variables
		{
			message << "Looking at action " << act_idx << endl;
			delta = 0; // overbodig: delta.push_back(0); //initialize vector delta of length number_of_actions to zeros
			delta = (mp_max_vals[act_idx] - mp_min_vals[act_idx]) * 
					(	pow (2,(mp_resolution[act_idx]-1) ) /
						(pow(2,mp_resolution[act_idx]) - 1)
					); //set the step size Delta for the current action variable
			message << "Delta = " << delta << endl;
			for(int depth = 0; depth< mp_resolution[act_idx]; depth++) //for all resolution bits of this variable
			{
				delta /= 2;
				//halve the step size
				double act_left = NN_input->at(l_state_dim + act_idx) - delta;
				double act_right = NN_input->at(l_state_dim + act_idx) + delta;
				message << "act_left = "<< act_left << endl;
				message << "act_right = "<< act_right << endl;

				NN_input->at(l_state_dim + act_idx) = act_left;
				try{
					double qval_left = QNN[act_idx]->forwardPropagate(NN_input)[0];

					NN_input->at(l_state_dim + act_idx) = act_right;
					double qval_right = QNN[act_idx]->forwardPropagate(NN_input)[0];

					// IS HET OOK HANDIG OM NAAR LOSSE ACTIE DIMENSIES TE KIJKEN??
				
					message << "Comparing at depth "<<depth<<". Left: "<<act_left<<" , right: "<<act_right<<".\n";
					message << "val left: " << qval_left << ". val right: "<<qval_right << endl;
					message << "Delta is: " << delta << endl;
					if (qval_left > qval_right){ //compare the two children
						//go left
						NN_input->at(l_state_dim + act_idx) = act_left;
						message << "Going left\n";
					}else{
						//go right
						//NN_input->at(l_state_dim + act_idx) = act_right; //not necessary, it is already at act_right
						message << "Going right\n";
					}
				} catch (int error)
				{
					cerr << "Forward propagate failed. Quitting";
					#ifdef WIN32
						char end;
						cin >> end;
					#endif
					//delete NN_input; //moet dit? of wordt dit afgehandeld door OS?
					exit(error);
				}
			} //Stuff at the end of one action-cycle
				
				//Debug log:
				message << "Final action[" << act_idx << "] : "<< NN_input->at(l_state_dim + act_idx)<< "\n\n";
				

				//Put the chosen actions in Action* action:
				//Is het misschien handig om hier hele kleine waarden af te ronden? bijv. output_action[act_idx] <0.001 wordt 0??
				action->continuousAction[act_idx] = NN_input->at(l_state_dim + act_idx);
		}
		//Stuff after both loops
		mp_BAS_log->write(message.str());
		delete NN_input;
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
		double* QTarget = new double[1]; //Hard coded that the output of the network is size 1

        if ( endOfEpisode ) {
			for(int act=0; act< action->actionDimension;act++)
			{
				QTarget[0] = rt;
				QNN[act]->backPropagate(st, QTarget, learningRate[0]) ;
			}

        } else {

			for(int act=0; act< action->actionDimension;act++)
			{
				double* Qt_ = QNN[act]->forwardPropagate( st_ );
				double* Qt = QNN[act]->forwardPropagate( st );
				QTarget[0] = rt + gamma*(Qt_[act]-Qt[act]) ;
				QNN[act]->backPropagate( st, QTarget, learningRate[0] ) ;
			}
        }

		//LOG CRITIC VALUE
		//stringstream value;
		//value	<< "output netwerk: " << Vt
		//		<< "\tTarget: " << QTarget[ 0 ]
		//		<< "\tReward: " << rt
		//		<< "\tGamma: " << gamma
		//		<< "\tVs_" << Vs_
		//		<< "\tVt_after " << Vt_after
		//		<< "\t learningRate[0] " << learningRate[0]
		//		<< "\t learningRate[1] " << learningRate[1];

		//mp_critic_log->write(value.str());
		delete[] QTarget;
	} else
		cerr << "Do not use discrete states!\n";

}

double BinaryActionSearch::updateAndReturnTDError( State * state, Action * action, double rt, State * state_,
								bool endOfEpisode, double * learningRate, double gamma  )								
{
	double * at = action->continuousAction ;
	double td_error;

	if(state->continuous)
	{
		double * st = state->continuousState ;
        double * st_ = state_->continuousState ; 
		double* QTarget = new double[1];
		double* Q_next_actions = new double[m_nr_actions];

		////Compute value of current state-action
		//double Qt = QNN[at]->forwardPropagate( st )[0];

        if ( endOfEpisode ) {

			for(int act=0; act< action->actionDimension;act++)
			{

			}

        } else {
			//

			//for(int act=0; act< action->actionDimension;act++)
			//{
			//	double Qt_ = QNN[act]->forwardPropagate( st_ )[0];
			//	Q_[act] = rt + gamma*Qt_ ;
			//}

			//	td_error += QTarget[act]-Qt;		//KLOPT DEZE TD ERROR WEL??
			//	QNN[act]->backPropagate( st, QTarget, learningRate[0] ) ;
			
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
	for(int action = 0; action < actionDimension; action++)
	{
		stringstream current_nn;
		current_nn << QNN_file << "_actionDim_" << action << ".txt";
		cNeuralNetwork* nn = new cNeuralNetwork(current_nn.str());
		QNN.push_back(nn);
	}
}

void BinaryActionSearch::writeQNN(std::string QNN_file)
{
	for(int action = 0; action < QNN.size(); action++){
		stringstream nn_file_name;
		nn_file_name << QNN_file << "_actionDim_"  << action << ".txt";
		QNN.at(action)->writeNetwork(nn_file_name.str());
	}
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
