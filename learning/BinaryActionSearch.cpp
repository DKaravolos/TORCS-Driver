#include "BinaryActionSearch.h"
#ifdef WIN32
	#include <time.h>
#endif

BinaryActionSearch::BinaryActionSearch( const char * parameterFile, World * w )
{
	_initVars(parameterFile,w);
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
		//Read NN parameters from file
		readParameterFile( parameterFile ) ;

		//Create Neural Network
        int layerSizesA[] = { stateDimension + actionDimension, nHiddenQ, 1 } ; //state dimension+1, omdat de actie erbij komt. hier kan extra laag
		//int QNN_settings[] = {0,1,1};

		//Create Networks for Q-values of going left and right
        for ( int a = 0 ; a < actionDimension ; a++ ) {
            QNN_left.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ; ///hier kan 1 ->2 voor extra laag
			QNN_right.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ; ///hier kan 1 ->2 voor extra laag
			srand(a+100+rand()); 
			QNN_right[a]->randomizeWeights(-0.3,0.3,rand());//THIS APPEARS NECESSARY FOR GETTING DIFFERENT RANDOM VALUES IN LEFT AND RIGHT
			srand(a+100+rand()); //THIS IS TO GET DIFFERENT VALUES in dim0 and dim1  
			QNN_left[a]->randomizeWeights(-0.3,0.3,rand()); //ALL OF THIS SHOULD NOT BE NECESSARY!
        }
    }
}

BinaryActionSearch::BinaryActionSearch(const char * parameterFile, World * w, string QNN_file)
{
	_initVars(parameterFile,w);
	if ( discreteStates ) {
		cerr << "Discrete states not implemented. Quitting.";
		#ifdef WIN32
			char end;
			cin >> end;
		#endif
		exit(-1);
    } else {
		//Create Neural Network
		cout << "QL: numberOfActions :" << actionDimension << endl;
		readQNN(QNN_file);
    }
}

void BinaryActionSearch::_initVars(const char* parameterFile, World* w)
{
	mp_BAS_log = new Writer("log_files/BAS_log.txt");
	m_NN_input = new vector<double>;

	//Init variables
    stateDimension = w->getStateDimension() ;
	actionDimension = w->getActionDimension() ;
	storedGauss = false ;
	mp_resolution =  new double[actionDimension];

	//Set the search resolution per action dimension
	//and initialise augmented state memory
	int general_resolution = 3;
	for(int act = 0; act < actionDimension; act++) {
		mp_resolution[act] = general_resolution;
	}

	//Create random seed
    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

}

BinaryActionSearch::~BinaryActionSearch(void)
{
	delete mp_BAS_log;

	delete[] mp_min_vals;
	delete[] mp_max_vals;
	delete[] mp_resolution;

	for(int act = 0; act < actionDimension; act++) {
		delete m_last_sequence[act];
		delete m_deltas[act];
		delete QNN_left[act];
		delete QNN_right[act];
	}
	m_last_sequence.clear();
	m_deltas.clear();
	QNN_left.clear();
	QNN_right.clear();
	delete m_NN_input;
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
	m_nr_actions = action->actionDimension; //m_nr_actions kan vervangen worden door actionDimension
	mp_min_vals = new double[m_nr_actions];
	mp_max_vals = new double[m_nr_actions];

	//Compute min and max vals
	for(int idx = 0; idx < m_nr_actions; idx++)
	{
		mp_min_vals[idx] = action->min_val[idx] ;
		mp_max_vals[idx] = action->max_val[idx] ;
		m_deltas.push_back(new vector<double>);
		m_last_sequence.push_back(new vector<double>);
	}

	//Precompute delta's
	for(int act_idx = 0; act_idx < m_nr_actions; act_idx++)
	{
		double delta = (mp_max_vals[act_idx] - mp_min_vals[act_idx]) * 
			(	pow (2,(mp_resolution[act_idx]-1) ) /
				(pow(2,mp_resolution[act_idx]) - 1)
			); //set the step size Delta for the current action variable

		//m_deltas[act_idx].reserve(mp_resolution[act_idx]);
		for(int depth = 0; depth < mp_resolution[act_idx]; depth++)
		{
			delta/=2;
			//m_deltas[act_idx][depth] = delta
			m_deltas[act_idx]->push_back(delta);
			m_last_sequence[act_idx]->push_back(0); //Initialize as zeros, so we can use access by index later.
		}
	}
}

//RL functions
void BinaryActionSearch::getMaxAction( State * state, Action * action )
{
	getMaxAction(state,action, ORIGINAL);
}

void BinaryActionSearch::getMaxAction( State * state, Action * action, GetMaxOption option)
{
	switch(option)
	{
		case ORIGINAL:
			originalGetMax(state,action);
			break;

		case INVERTED_LOOPS:
			invertedLoopsGetMax(state,action);
			break;

		default:
			cerr << "Unknown GetMaxOption, please check its value. Calling original getMaxAction.\n";
			originalGetMax(state,action);
	}
}

void BinaryActionSearch::originalGetMax( State * state, Action * action )
{
	stringstream message;
	if(action->continuous)
	{
		//Create input for neural network
		int l_inputsize = stateDimension + actionDimension;
		for(int idx = 0; idx < stateDimension; idx++)
		{
			m_NN_input->push_back(state->continuousState[idx]);
		}

		//Init action variables for binary search
		for(int act_idx = 0; act_idx < actionDimension; act_idx++) //iterate over action variables
		{	
			double l_mid_range = (mp_max_vals[act_idx] + mp_min_vals[act_idx]) / 2;
			m_NN_input->push_back(l_mid_range); //init each action variable to middle of its range
		}

		//Do the actual search
		for(int act_idx = 0; act_idx < actionDimension; act_idx++) //iterate over action variables
		{
			//message << "Looking at action " << act_idx << endl;
			double delta;
			for(int depth = 0; depth< mp_resolution[act_idx]; depth++) //for all resolution bits of this variable
			{
				//halve the step size
				delta = m_deltas[act_idx]->at(depth);
				
				try{
					double qval_left = QNN_left[act_idx]->forwardPropagate(m_NN_input)[0]; //Left cooresponds to choosing -delta
					double qval_right = QNN_right[act_idx]->forwardPropagate(m_NN_input)[0]; //right cooresponds to choosing +delta

					//message << "Comparing at depth "<<depth<<".\n";
					//message << "val left: " << qval_left << ". val right: "<<qval_right << endl;
					//message << "Delta is: " << delta << endl;
					if (qval_left > qval_right){ //compare the two children
						//go left
						m_NN_input->at(stateDimension + act_idx) -= delta;
						m_last_sequence[act_idx]->at(depth) = -delta;
						//message << "Going left\n";
						//message << "m_last_sequence[" << act_idx << "]["<<depth << "] =" << m_last_sequence[act_idx]->at(depth) << endl;
						//message << "next node: " << m_NN_input->at(stateDimension + act_idx);
					}else{
						//go right
						m_NN_input->at(stateDimension + act_idx) += delta;
						m_last_sequence[act_idx]->at(depth) = delta;
						//message << "Going right\n";
						//message << "m_last_sequence[" << act_idx << "]["<<depth << "] =" << m_last_sequence[act_idx]->at(depth) << endl;
						//message << "next node: " << m_NN_input->at(stateDimension + act_idx);
					}
				} catch (int error)
				{
					cerr << "Forward propagate failed. Quitting";
					#ifdef WIN32
						char end;
						cin >> end;
					#endif
					exit(error);
				}
			}
			//Stuff at the end of one action-cycle
			//Debug log:
			message << "\nFinal action[" << act_idx << "] : "<< m_NN_input->at(stateDimension + act_idx)<< "\n\n";
				

			//Put the chosen actions in Action* action:
			//Is het misschien handig om hier hele kleine waarden af te ronden? bijv. output_action[act_idx] <0.001 wordt 0??
			action->continuousAction[act_idx] = m_NN_input->at(stateDimension + act_idx);
		}
		//Stuff after both loops
		mp_BAS_log->write(message.str());
		m_NN_input->clear();
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

void BinaryActionSearch::invertedLoopsGetMax( State * state, Action * action )
{
	stringstream message;
	if(action->continuous)
	{
		//Create input for neural network
		int l_inputsize = stateDimension + actionDimension;
		for(int idx = 0; idx < stateDimension; idx++)
		{
			m_NN_input->push_back(state->continuousState[idx]);
		}

		//Init action variables for binary search
		for(int act_idx = 0; act_idx < actionDimension; act_idx++) //iterate over action variables
		{	
			double l_mid_range = (mp_max_vals[act_idx] + mp_min_vals[act_idx]) / 2;
			m_NN_input->push_back(l_mid_range); //init each action variable to middle of its range
		}

		//Do the actual search
		//WE NOW ASSUME EQUAL RESOLUTION OVER ALL ACTION VARIABLES
		//VARIABLE RESOLUTION IS DIFFICULT TO CODE AND NOT THE FOCUS ATM
		for(int depth = 0; depth <  mp_resolution[0]; depth++) //for all resolution bits of this variable
		{
			message << "Resolution: " << depth << endl;
			double delta;
			for(int act_idx = 0; act_idx < actionDimension; act_idx++) //iterate over action variables
			{
				//halve the step size
				delta = m_deltas[act_idx]->at(depth);
				
				try{
					double qval_left = QNN_left[act_idx]->forwardPropagate(m_NN_input)[0]; //Left cooresponds to choosing -delta
					double qval_right = QNN_right[act_idx]->forwardPropagate(m_NN_input)[0]; //right cooresponds to choosing +delta

					if (qval_left > qval_right){ //compare the two children
						//go left
						m_NN_input->at(stateDimension + act_idx) -= delta;
						m_last_sequence[act_idx]->at(depth) = -delta;
					}else{
						//go right
						m_NN_input->at(stateDimension + act_idx) += delta;
						m_last_sequence[act_idx]->at(depth) = delta;
					}
				} catch (int error)
				{
					cerr << "Forward propagate failed. Quitting";
					#ifdef WIN32
						char end;
						cin >> end;
					#endif
					exit(error);
				}
			}//end of action-cycles
		} //end of resolution-cycles

		//Put the chosen actions in Action* action:
		//Is het misschien handig om hier hele kleine waarden af te ronden? bijv. output_action[act_idx] <0.001 wordt 0??
		for(int act_idx = 0; act_idx < actionDimension; act_idx++)
			action->continuousAction[act_idx] = m_NN_input->at(stateDimension + act_idx);

		mp_BAS_log->write(message.str());
		m_NN_input->clear();
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
	double * l_action = action->continuousAction ;
	if(state->continuous)
	{
		double* QTarget = new double[1]; //Hard coded that the output of the network is size 1

		//set NN representation of this state and next state
		int input_size = stateDimension + actionDimension;
		double* NN_input = new double[input_size];
		double* NN_input_prime = new double[input_size];
		for(int idx = 0; idx < stateDimension; idx++)
		{
			NN_input[idx] = state->continuousState[idx]; //values of current state
			NN_input_prime[idx] = state_->continuousState[idx]; //values of next state
		}
		for(int idx = 0; idx < actionDimension; idx++)
		{
			NN_input[stateDimension+idx] = l_action[idx];
			//hard coded, we know this is the middle of the range of all action dimensions:
			NN_input_prime[stateDimension+idx] = 0; 
		}


		//Do the updates
		for(int act=0; act < actionDimension; act++) //loop through actions
		{
			if ( endOfEpisode )
			{
				QTarget[0] = rt;
			} else
			{
				//Compute QTarget:
				//Compute max Q-val of next state
				double next_q;
				double qval_left = QNN_left[act]->forwardPropagate(NN_input_prime)[0];
				double qval_right = QNN_right[act]->forwardPropagate(NN_input_prime)[0];
				
				qval_left > qval_right? next_q = qval_left: next_q = qval_right;
				QTarget[0] = rt + gamma * next_q;
			}
				/*First: update actual action towards QTarget */

				//reverse last step to get to intermediate state-action value
				double l_last_step = m_last_sequence[act]->at(m_last_sequence.size()-1);
				NN_input[stateDimension + act] -= l_last_step; 

				//Do update:
				//Update the network that did the last step
				if( l_last_step < 0)
					QNN_left[act]->backPropagate(NN_input, QTarget, learningRate[0]) ;
				else if (l_last_step > 0)
					QNN_right[act]->backPropagate(NN_input, QTarget, learningRate[0]) ;
				else 
					cerr << "\n\nBAS: Last step of zero should be impossible. Something is wrong.\n\n";

				/* Then: loop through augmented state-actions and update towards Q-value of previous state-action node */
				updateAugmentedStates(NN_input, act, learningRate[0]);
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
		delete[] NN_input;
		delete[] NN_input_prime;
	} else
		cerr << "Do not use discrete states!\n";

}

double BinaryActionSearch::updateAndReturnTDError( State * state, Action * action, double rt, State * state_,
								bool endOfEpisode, double * learningRate, double gamma  )								
{
	double * l_action = action->continuousAction ;
	double td_error;
	if(state->continuous)
	{
		double* QTarget = new double[1]; //Hard coded that the output of the network is size 1

		//set NN representation of this state and next state
		int input_size = stateDimension + actionDimension;
		double* NN_input = new double[input_size];
		double* NN_input_prime = new double[input_size];
		for(int idx = 0; idx < stateDimension; idx++)
		{
			NN_input[idx] = state->continuousState[idx]; //values of current state
			NN_input_prime[idx] = state_->continuousState[idx]; //values of next state
		}
		for(int idx = 0; idx < actionDimension; idx++)
		{
			NN_input[stateDimension+idx] = l_action[idx];
			//hard coded, we know this is the middle of the range of all action dimensions:
			NN_input_prime[stateDimension+idx] = 0; 
		}

		//Do the updates

		for(int act=0; act < actionDimension; act++) //loop through actions
		{
			if ( endOfEpisode )
			{
				QTarget[0] = rt;
			} else
			{
				//Compute QTarget:
				//Compute max Q-val of next state
				double next_q;
				double qval_left = QNN_left[act]->forwardPropagate(NN_input_prime)[0];
				double qval_right = QNN_right[act]->forwardPropagate(NN_input_prime)[0];
				
				qval_left > qval_right? next_q = qval_left: next_q = qval_right;
				QTarget[0] = rt + gamma * next_q;
			}
				/*First: update actual action towards QTarget */

				//reverse last step to get to intermediate state-action value
				double l_last_step = m_last_sequence[act]->at(m_last_sequence.size()-1);
				NN_input[stateDimension + act] -= l_last_step; 

				//Do update:
				//Update the network that did the last step
				if( l_last_step < 0)
					QNN_left[act]->backPropagate(NN_input, QTarget, learningRate[0]) ;
				else if (l_last_step < 0)
					QNN_right[act]->backPropagate(NN_input, QTarget, learningRate[0]) ;
				else 
					cerr << "\n\nBAS: Last step of zero should be impossible. Something is wrong.\n\n";

				/* Then: loop through augmented state-actions and update towards Q-value of previous state-action node */
				updateAugmentedStates(NN_input, act, learningRate[0]);
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
		delete[] NN_input;
		delete[] NN_input_prime;
	} else
		cerr << "Do not use discrete states!\n";

	return td_error;
}

//Description:
void BinaryActionSearch::updateAugmentedStates(double* NN_input, int act, double learningRate)
{
	double l_action = NN_input[stateDimension + act];
	double l_last_step;
	//loop backwards through sequence of augmented space
	for(int depth = m_last_sequence.size()-2;  depth >= 0; --depth) 
	{

		l_last_step = m_last_sequence[act]->at(depth); //has a signed delta inside

		//Get the value of the max action (i.e. val of nodes on the lower layer)
		double* l_qtarget = NULL;
		double qval_left = QNN_left[act]->forwardPropagate(NN_input)[0];
		double qval_right = QNN_right[act]->forwardPropagate(NN_input)[0];

		if( qval_left > qval_right)
			l_qtarget = QNN_left[act]->forwardPropagate(NN_input);
		else
			l_qtarget = QNN_right[act]->forwardPropagate(NN_input);

		//change NN_input to be at the right node in the tree.
		NN_input[stateDimension + act] -= l_last_step; 
					
		//Update the Qval of the action in this layer (i.e. relevant neural network for left or right action)
		// target = r+ gmma* max(Q(next state action pair,max network)
		// where rt = 0 and gamma = 1, so just the Qval of the next network with given state-action pair.
		if(m_last_sequence[act]->at(depth) < 0)
		{
			QNN_left[act]->backPropagate(NN_input, l_qtarget, learningRate) ;
		} else
		{
			QNN_right[act]->backPropagate(NN_input, l_qtarget, learningRate) ;
		}
	}
	//end of augmented-loop stuff:
	//Reset action input to original value
	NN_input[stateDimension + act] = l_action;
}

//Neural Network functions
void BinaryActionSearch::readQNN(std::string QNN_file)
{
	for(int action = 0; action < actionDimension; action++)
	{
		stringstream current_nn;
		current_nn << QNN_file << "_actionDim_" << action;
		
		QNN_left.push_back(new cNeuralNetwork(current_nn.str() + "_left.txt"));
		QNN_right.push_back(new cNeuralNetwork(current_nn.str() + "_right.txt"));
	}
}

void BinaryActionSearch::writeQNN(std::string QNN_file)
{
	for(int action = 0; action < QNN_left.size(); action++){
		stringstream nn_file_name;
		nn_file_name << QNN_file << "_actionDim_"  << action;
		QNN_left.at(action)->writeNetwork(nn_file_name.str() + "_left.txt");
		QNN_right.at(action)->writeNetwork(nn_file_name.str() + "_right.txt");
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
