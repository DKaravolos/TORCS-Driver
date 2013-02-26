#include "BASWithRoots.h"

BASWithRoots::BASWithRoots(const char * parameterFile, World * w):
BinaryActionSearch(parameterFile,w)
{
	//Create Neural Network
    int layerSizesA[] = { stateDimension + actionDimension, nHiddenQ, 1 } ; //state dimension+1, omdat de actie erbij komt. hier kan extra laag

	//Create Networks for Q-values of staying at a node in the search tree
    for ( int a = 0 ; a < actionDimension ; a++ ) {
        QNN_stay.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ; ///hier kan 1 ->2 voor extra laag
		srand(a+100+rand()); //THIS APPEARS NECESSARY FOR GETTING DIFFERENT RANDOM VALUES IN DIM 0 AND 1
		QNN_stay[a]->randomizeWeights(-0.3,0.3, rand()); //IT SHOULD NOT BE NECESSARY THOUGH!
    }
}

BASWithRoots::BASWithRoots(const char * parameterFile, World * w, string QNN_file):
BinaryActionSearch(parameterFile,w, QNN_file)
{
	initQStay(QNN_file);
}

BASWithRoots::~BASWithRoots()
{
	for(int act = 0; act < actionDimension; act++) {
		delete QNN_stay[act];
	}

	QNN_stay.clear();
}


void BASWithRoots::getMaxAction (State * state, Action * action)
{
	withRootsGetMax(state,action);
}

void BASWithRoots::withRootsGetMax (State * state, Action * action)
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
			double delta;
			for(int depth = 0; depth< mp_resolution[act_idx]; depth++) //for all resolution bits of this variable
			{
				if(depth > 0 && m_last_sequence[act_idx]->at(depth-1) == 0){
					m_last_sequence[act_idx]->at(depth) = 0;
					message << "M - due to continue" <<endl;
					continue; //if rootnode is chosen, don't continue searching, otherwise the move becomes irrelevant. (just another way to get to a certain point)
				}
				//halve the step size
				delta = m_deltas[act_idx]->at(depth);
				//delta /= 2;
				
				try{
					double qval_root = QNN_right[act_idx]->forwardPropagate(m_NN_input)[0]; //Root corresponds to choosing current root as action
					double qval_left = QNN_left[act_idx]->forwardPropagate(m_NN_input)[0]; //Left corresponds to choosing -delta
					double qval_right = QNN_right[act_idx]->forwardPropagate(m_NN_input)[0]; //right corresponds to choosing +delta

					message << "Comparing at depth "<<depth<<".\n";
					message << "val root = " << qval_root << ". val left: " << qval_left << ". val right: "<<qval_right << endl;
					message << "Delta is: " << delta << endl;

					if (qval_left > qval_root) //compare the two children
					{ 
						if(qval_left > qval_right)
						{
							//go left
							m_NN_input->at(stateDimension + act_idx) -= delta;
							m_last_sequence[act_idx]->at(depth) = -delta;
							message << "L" <<endl;

						} else {
							//go right
							m_NN_input->at(stateDimension + act_idx) += delta;
							m_last_sequence[act_idx]->at(depth) = delta;
							message << "R" <<endl;
						}
					} else if (qval_right > qval_root)
					{
						//go right
						m_NN_input->at(stateDimension + act_idx) += delta;
						m_last_sequence[act_idx]->at(depth) = delta;
						message << "R" <<endl;
					} else {
						//choose root
						//No change to m_NN_input->at(stateDimension + act_idx)
						m_last_sequence[act_idx]->at(depth) = 0;
						message << "M" <<endl;
					}


					//if (qval_left > qval_right) //compare the two children
					//{ 
					//	if(qval_left > qval_root)
					//	{
					//		//go left
					//		m_NN_input->at(stateDimension + act_idx) -= delta;
					//		m_last_sequence[act_idx]->at(depth) = -delta;
					//	} else {
					//		//choose root
					//		//No change to m_NN_input->at(stateDimension + act_idx)
					//		m_last_sequence[act_idx]->at(depth) = 0;
					//	}
					//}else if (qval_right > qval_left)
					//{
					//	if(qval_right > qval_root)
					//	{
					//		//go right
					//		m_NN_input->at(stateDimension + act_idx) += delta;
					//		m_last_sequence[act_idx]->at(depth) = delta;
					//	} else {
					//		//choose root
					//		//No change to m_NN_input->at(stateDimension + act_idx)
					//		m_last_sequence[act_idx]->at(depth) = 0;
					//	}
					//} else {
					//	//choose root
					//	//No change to m_NN_input->at(stateDimension + act_idx)
					//	m_last_sequence[act_idx]->at(depth) = 0;
					//}

				} catch (int error)
				{
					cerr << "Forward propagate failed. Quitting";
					#ifdef WIN32
						char end;
						cin >> end;
					#endif
					exit(error);
				}
			} //Stuff at the end of one action-cycle
				
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

void BASWithRoots::update( State * state, Action * action, double rt, State * state_, bool endOfEpisode, double * learningRate, double gamma  )
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
				double qval_root = QNN_stay[act]->forwardPropagate(NN_input_prime)[0];
				double qval_right = QNN_right[act]->forwardPropagate(NN_input_prime)[0];

				
				if (qval_left > qval_root) //Decide what is the max action of 3 possibilities
				{ 
					if(qval_left > qval_right)
					{
						next_q = qval_left;
					} else {
						next_q = qval_right;
					}
				} else if (qval_right > qval_root)
				{
					next_q = qval_right;
				} else {
					next_q = qval_root;
				}
				
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
					QNN_stay[act]->backPropagate(NN_input, QTarget, learningRate[0]) ;

				/* Then: loop through augmented state-actions and update towards Q-value of previous state-action node */
				//updateAugmentedStates(NN_input, act, learningRate[0]);
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

//Neural Network functions
void BASWithRoots::readQNN(std::string QNN_file)
{
	for(int action = 0; action < actionDimension; action++)
	{
		stringstream current_nn;
		current_nn << QNN_file << "_actionDim_" << action;
		
		QNN_left.push_back(new cNeuralNetwork(current_nn.str() + "_left.txt"));
		QNN_stay.push_back(new cNeuralNetwork(current_nn.str() + "_stay.txt"));
		QNN_right.push_back(new cNeuralNetwork(current_nn.str() + "_right.txt"));
	}
}

void BASWithRoots::initQStay(std::string QNN_file)
{
	for(int action = 0; action < actionDimension; action++)
	{
		stringstream current_nn;
		current_nn << QNN_file << "_actionDim_" << action;
		QNN_stay.push_back(new cNeuralNetwork(current_nn.str() + "_stay.txt"));
	}
}

void BASWithRoots::writeQNN(std::string QNN_file)
{
	for(unsigned int action = 0; action < QNN_stay.size(); action++){
		stringstream nn_file_name;
		nn_file_name << QNN_file << "_actionDim_"  << action;
		QNN_left.at(action)->writeNetwork(nn_file_name.str() + "_left.txt");
		QNN_stay.at(action)->writeNetwork(nn_file_name.str() + "_stay.txt");
		QNN_right.at(action)->writeNetwork(nn_file_name.str() + "_right.txt");
	}
}