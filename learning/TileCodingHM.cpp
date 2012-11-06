# include "TileCodingHM.h"
#define DEFAULT_Q 50.0 //Defines the default (i.e. first) value of a tile


TileCodingHM::TileCodingHM(World * w, const string& log_dir)
{
	init(w, log_dir);    
}

TileCodingHM::TileCodingHM(World * w, const string& log_dir, const char* qtable_file)
{
	loadQTable(string(qtable_file));
	init(w, log_dir);
}

TileCodingHM::~TileCodingHM()
{
	delete mp_log;
}

//initialisation
void TileCodingHM::init(World* w, const string& log_dir)
{
	discreteStates = w->getDiscreteStates() ;

    if (!w->getDiscreteActions() || w->getDiscreteStates()) {

        cout << "This TileCodingHM does not support continuous actions or discrete states." << endl ;
        cout << "Please check which MDP you are using it on." << endl ;
		#ifdef WIN32
			char end_program;
			cin >> end_program;
		#endif
		exit(-4) ;

    } else {
        numberOfActions = w->getNumberOfActions() ;
    }
	stateDimension = w->getStateDimension();


	mp_log = new Writer(log_dir + "TileCodingLog.txt"); // Creating the log file does not depend on answer
	//Get parameter/config info from user ??
	bool test = false;
	if(test)
		cout << "TileCoding: Using test settings!\n";

	if(log_dir.compare("log_files/") == 0 && !test) //LET OP: ALLEEN VOOR TEST Driver!!
	{
		cout << "Do you want a verbose log file for the TileCoding algorithm?\n";
		char ans;
		cin >> ans;
		m_verbose = (ans =='y');

		//Set number of tilings
		cout << "How many tilings are defined in the edge configuration file?\n";
		cin >> m_numTilings; //NOTE: Vector subscript out of range error is produced when the actual number is larger than this.
	} else {
		m_verbose = false;
		m_numTilings = 10; //HARD CODED SETTING! LAZY PROGRAMMER ALERT!
	}

	//Set edges of squares in tiles
	setEdges();

	//Set size of state-action vectors
	m_state_keys.resize(m_numTilings);
	m_next_state_keys.resize(m_numTilings);
}

//Sets the edges of the bins of the tiles using file input
void TileCodingHM::setEdges()
{
	//init sizes, so we can use element access of vectors 
	m_speed_edges.resize(m_numTilings);
	m_trackPos_edges.resize(m_numTilings);
	m_angle_edges.resize(m_numTilings);
	m_dist_edges.resize(m_numTilings);

	//cout << "Hoeveel vakjes gebruiken je tilings?\n";
	int num = 4;
	//cin >> num;
	stringstream filename;
	filename << "TileCodingEdges_5Tiles_"<< num << "Vakjes.txt";
	getEdgesFromFile(filename.str()); // assumes to find a maximum of m_numTilings values for tilings
	
	cout << "JE GEBRUIKT " << num << " VAKJES IN EEN TILING.\n";
}

//Reads the edges of the bins of the tiles from a file
void TileCodingHM::getEdgesFromFile(string l_parameterfile)
{
	//Keep track of the defined tilings per dimension
	int speed_tiling = 0;
	int pos_tiling = 0;
	int angle_tiling = 0;
	int dist_tiling = 0;

	//Let's open the file
	string line;
	ifstream is;
	is.open(l_parameterfile);
	if(is.is_open())
	{
		try
		{
			while(!is.eof()) //We could place the getline in the while-condition, but an eof-check seems nicer
			{
				//Get line from file
				getline(is, line); 

				//Temporary variables
				stringstream edges(line);
				string edge_type;

				//Classify the line by the first item
				edges >> edge_type;
				if(edge_type.compare("speed") == 0)
				{
					addEdge(edge_type, edges, m_speed_edges, speed_tiling);
				}
				else if (edge_type.compare("pos") == 0)
				{
					addEdge(edge_type, edges, m_trackPos_edges, pos_tiling);
				}
				else if (edge_type.compare("angle") == 0)
				{
					addEdge(edge_type, edges, m_angle_edges, angle_tiling);
				}
				else if (edge_type.compare("dist") == 0)
				{
					addEdge(edge_type, edges, m_dist_edges, dist_tiling);
				}
			}

			if (speed_tiling == pos_tiling &&
				speed_tiling == angle_tiling &&
				speed_tiling == dist_tiling)
			{
				cout << "\nYou have defined "<< speed_tiling << " tilings.\n";

				if(speed_tiling != m_numTilings)
				{
					//cout << "This is not equal to " << m_numTilings << ". Continue? (y/n)\n";
					//char ans;
					//cin >> ans;
					//if(ans == 'n')
					//	exit(0);
					//else
						m_numTilings = speed_tiling;
				}

				//Set tile size (assuming that the user has created tilings of equal size)
				m_tileNum_speed = m_speed_edges[0].size(); //note: this is the number of tiles per tiling
				m_tileNum_pos = m_trackPos_edges[0].size();
				m_tileNum_dist = m_dist_edges[0].size();
				m_tileNum_angle = m_angle_edges[0].size();

			} else {
				cerr << "\nOh no! You seem to have defined a different number of tilings in some dimensions!\n";
				cerr << "# of speed tilings: " << speed_tiling << endl;
				cerr << "# of trackpos tilings: " << pos_tiling << endl;
				cerr << "# of angle tilings: " << angle_tiling << endl;
				cerr << "# of distance tilings: " << dist_tiling << endl;
			}
						
		} catch (iostream::failure e)
		{
			cerr << "Failure during reading edges from file.\n";
			cerr << e.what();
		}
		is.close();

	} else {
		cerr <<  "Can't open tile coding parameter file. Quitting\n";
		#ifdef WIN32
			char end_program;
			cin >> end_program;
		#endif
		exit(-8) ;
	}
}

//Adds edge from input to given vector
void TileCodingHM::addEdge(string& edgetype, stringstream& ss, vector<vector<double>>& edge_vector, int& tiling)
{
	double edge;
	//It would be nice to be able to check if the number of tilings does not exceed user input
	cout << "Reading " << edgetype << " edges of tiling \t\t" << tiling << ": ";
	//Add the rest of the line to the edge vector
	while(ss >> edge){
		edge_vector[tiling].push_back(edge); //Note: we don't want a double push_back! (then the edges don't end up in the same tiling)
		cout << edge << "  ";
	}
	cout << endl;
	++tiling; //Increase the tiling count 
}

//same as updateAndReturnTDError, but does not return TD error
void TileCodingHM::update( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma)
{
	updateAndReturnTDError(state,action,rt,nextState,endOfEpisode,learningRate,gamma);
}

//Performs an update of the Q-table by computing the TD error explicitly and returns the TD error.
double TileCodingHM::updateAndReturnTDError( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma)
{
	if (!state->continuous )
	{
		cerr << "Tile coding does not support discrete states!! Can't update!\n";
		return 0;
	}
	
	int current_action = action->discreteAction;

	stringstream l_out;
	if(m_verbose)
		l_out << "Start of update()\n";
	
	//For each tile: convert continuous state into discrete states defined by tiles
	double active_tilings = 0;
	int tiling;
	for(tiling = 0; tiling < m_numTilings; tiling++)
	{
		//Identify the active tiles of this state
		m_state_keys[tiling] = classifyState(state, tiling);
		//cout << "State key: " << m_state_keys[tiling] << endl;

		if(m_state_keys[tiling].empty() == false)
			active_tilings++;
		else
			cerr << "ERROR: There is an inactive tiling in the current state.\n";

		//Store the visit to this state
		pair<string, int> state_action = make_pair(m_state_keys[tiling], current_action);
		storeStateVisit(state_action);
	}	
	//l_out << "Current state : t0 =" << m_state_keys[0] << ", t1 = " << m_state_keys[1] << endl;


	//Define the learning rate of this update. This depends on the number of active tilings.
	double learning_rate = learningRate[0] / active_tilings;

	//For each tile: convert continuous next_state into discrete next_states defined by tiles
	for(int tiling = 0; tiling < m_numTilings; tiling++)
	{
		m_next_state_keys[tiling] = classifyState(nextState, tiling);
		if(m_next_state_keys[tiling].empty() == true)
			cerr << "ERROR: There is an inactive tiling in the next state.\n";
	}
	if(m_verbose)
		l_out << "Taken action: " << current_action <<endl;
	//mp_log->write(l_out.str());
	//l_out.str("");
	//l_out.clear();

	double q_of_state = getQOfStateActionPair(m_state_keys, current_action);
	double td_error;

	//l_out << "Next state : t0 =" << m_next_state_keys[0] << ", t1 = " << m_next_state_keys[1] << endl;
	//l_out << "Learning rate: " << learning_rate << endl;
	if(m_verbose) {
		l_out << "Reward: " << rt << endl;
		l_out << "Q-value of state-action pair: " << q_of_state << endl;
		mp_log->write(l_out.str());
		l_out.str("");
		l_out.clear();
	}

	if(endOfEpisode)
	{
		//Add penalty to end of episode state
		//rt -= 10;
		//cout << "NOTE: in TC-HM a reward of -10 is added for end of episode\n";

		//Compute TD error (independent of tiling)
		td_error = rt - q_of_state;
		if(m_verbose)
			l_out << "TD Error: " << td_error << "( rt - q_of_state )" << endl;
		////Update the Q-values for each active tile (per tiling) depending on the reward
		//for each theta: theta[i][a] = theta[i][a] + lr * (rt- phi[i][a] * theta[i][a]) * phi[i][a]
		for(int tiling = 0; tiling < m_numTilings; tiling++)
		{
			if(m_state_keys[tiling].empty() == false) //no update if the tiling has no active feature (this filters the second phi[i][a] if phi[i][a] = 0 )
			{
				//l_out << "Updating: state " << m_state_keys[tiling] << "(Tiling " << tiling << "), Action: " << current_action << endl;

				//Only update the active feature in a tiling. (this filters the first phi[i][a] if phi[i][a] = 0)
				pair<string,int> state_action = make_pair (m_state_keys[tiling], current_action);
				//l_out << "Old value: " << m_tilings[state_action] << endl;
				m_tilings[state_action]= learning_rate * td_error;
				
				//l_out << " New value: " << m_tilings[state_action] << endl;
			}
		}
	} else
	{
		//Compute max action of next state (independent of tiling)
		double max_q = getMaxQOfState(m_next_state_keys);

		//Compute TD error (independent of tiling)
		td_error = (rt + gamma * max_q) - q_of_state;

		if(m_verbose)
		{
			l_out << "Q-value of best action in next state: " << max_q << endl;
			l_out << "TD error: " << td_error << " (rt + gamma * max_q - q_of_state)\n";
		}
		////Update the Q-values for each active tile (per tiling) depending on the reward and the next state
		//for each theta: theta[i][a] = theta[i][a] + lr * (rt + gmma * maxQs - phi[i][a] * theta[i][a]) * phi[i][a]
		//// note: i = tiling, a = at (=action)
		for(int tiling = 0; tiling < m_numTilings; tiling++)
		{
			if(m_state_keys[tiling].empty() == false && m_next_state_keys[tiling].empty() == false) //no update if the tiling has no active feature (this filters the second phi[i][a] if phi[i][a] = 0 )
			{
				//Only update the active feature in a tiling. (this filters the first phi[i][a] if phi[i][a] = 0)
				pair<string,int> state_action = make_pair (m_state_keys[tiling], current_action);

				//l_out << "Updating: state " << m_state_keys[tiling] << "(Tiling " << tiling << "), Action: " << current_action << endl;
				//l_out << "Old value: " << m_tilings[state_action] << endl;
				m_tilings[state_action]	+= learning_rate * td_error;
				//l_out << "New value: " << m_tilings[state_action] << endl;

			} else {
				cerr << "This state is not classified by tiling " << tiling << ", update will be partial (and thus wrong).\n"; 
			}
		}		
	}

	if(m_verbose)
	{
		l_out << "\n\n";
		mp_log->write(l_out.str());
	}

	for(tiling = 0; tiling < m_numTilings; tiling++)
	{
		pair<string, int> state_action = make_pair(m_state_keys[tiling], current_action);
		storeAverageTDError(state_action, td_error);
	}
	return td_error;
}

// Find the tile indices for a given state. Returns NULL if the state is not in this tile.
string TileCodingHM::classifyState(const State* state, const int& tiling)
{
	stringstream key;
	int tile_indices[8]; //NOTE: It should be possible to use a vector<int>* for this


	//stringstream log;
	//log << "Classifying state with tiling " << tiling;
	//mp_log->write(log.str());
	//cout << "classifying tiling "<< tiling << endl;
	//Classify each dimension separately
	//mp_log->write("Classifying speed.");
	tile_indices[0] = classifyValue(state->continuousState[0], m_speed_edges[tiling]);
	//mp_log->write("Classifying trackPos.");
	tile_indices[1] = classifyValue(state->continuousState[1], m_trackPos_edges[tiling]);
	//mp_log->write("Classifying Angle.");
	tile_indices[2] = classifyValue(state->continuousState[2], m_angle_edges[tiling]);
	//cout << "Angle. Value: " << state->continuousState[2] << " tile: " << tile_indices[2] << endl;
	//mp_log->write("Classifying Dist1.");
	tile_indices[3] = classifyValue(state->continuousState[3], m_dist_edges[tiling]);
	//mp_log->write("Classifying Dist2.");
	tile_indices[4] = classifyValue(state->continuousState[4], m_dist_edges[tiling]);
	//mp_log->write("Classifying Dist3.");
	tile_indices[5] = classifyValue(state->continuousState[5], m_dist_edges[tiling]);
	//mp_log->write("Classifying Dist4.");
	tile_indices[6] = classifyValue(state->continuousState[6], m_dist_edges[tiling]);
	//mp_log->write("Classifying Dist5.");
	tile_indices[7] = classifyValue(state->continuousState[7], m_dist_edges[tiling]);
	
	//use tiling as part of key for map
	key << tiling;

	//Check if there was any value out of bounds
	int idx;
	for(idx = 0; idx < 8; ++idx)
	{
		if(tile_indices[idx] >= 0)
		{
			key << tile_indices[idx]; //if not, continue creating key
		} else { //If any index is out of bounds, the state is not in this tiling
			cerr << "Oh no! Dimension " << idx << " does not fit in a tile of tiling " << tiling << endl;
			return ""; // so return an empty key, which creates an error 			
		}
	}
	//mp_log->write("Resulting key:");
	//mp_log->write(key.str());
	return key.str();
}

//Classifies a given value into the given tile bins
int TileCodingHM::classifyValue(const double& state_value, const vector<double>& bin_edges)
{
	//stringstream log;
	int bin = -1; 
	for(unsigned int idx = 0; idx < bin_edges.size()-1; idx++)
		if(state_value >= bin_edges[idx] && state_value < bin_edges[idx+1]) //check each bin
		{
			//cout << "Value " << state_value << " is between " << bin_edges[idx] << " and " << bin_edges[idx+1];
			//cout << ", thus the bin is " << idx << endl;
			//log << "Value " << state_value << " is between " << bin_edges[idx] << " and " << bin_edges[idx+1];
			//log << ", thus the bin is " << idx << endl;
			bin = idx;
			break;
		}
	//mp_log->write(log.str());
	return bin; //If the state_value is outside the tile range, it returns -1. 
				//This is sort of an exception code, it should be caught later, because indices cannot be -1
}

//Gets the max action-value of a state (vector of strings)
double TileCodingHM::getMaxQOfState(const vector<string>& state_keys)
{
	vector<double> Q_values = vector<double> (numberOfActions, 0.0);
	//stringstream l_out;
	//l_out << "GetMaxQOfState\n";
	//Find the Q-value of all actions given the tiles
	for(int tiling = 0; tiling < m_numTilings; ++tiling)
	{
		//l_out << "Loop through actions..\n";
		for(int act = 0; act < numberOfActions; ++act)
		{
			if(state_keys[tiling].empty() == false) //If the tile can exist
			{
				pair<string,int> state_action = make_pair(state_keys[tiling],act);
				map<pair<string,int>,double>::iterator it = m_tilings.find(state_action);
				if(it != m_tilings.end()) //If the state-action pair is already in the map
				{
					////l_out << "Known Key " << it->first.first << it->first.second <<" in tiling " << tiling <<". Value: "<< it->second << endl;
					Q_values[act] += it->second; //add found value to Q-value
				} else {				//else
					m_tilings[state_action] = DEFAULT_Q; //Set the value to default
					////l_out << "New Key: " << state_action.first << state_action.second;
					////l_out << ". Value: " << DEFAULT_Q << endl;
					Q_values[act] += m_tilings[state_action]; //Add the value of the newly created entry to the Q-value
				}
			} else {
				cerr << "ERROR: state_keys ["<<tiling<<"] is empty!!\n";
				//l_out << "ERROR: state_keys ["<<tiling<<"] is empty!!\n";
			}
		}
	}
	//l_out << "Q-values of actions in state '" << state_keys[0] << "' and '" << state_keys[1] <<"'\n";
	//for(int idx=0; idx < Q_values.size(); ++idx)
	//{
	//	l_out << "Action "<< idx << ": " << Q_values[idx] << endl;
	//}

	//l_out << "End of GetMaxQOfState\n";
	//mp_log->write(l_out.str());
	return myTCMax(Q_values);
}

//Gets the Q-value of a specific tile
double TileCodingHM::getQOfStateActionPair(const vector<string>& state_keys, int action)
{
	double Q_value = 0;
	int tiling;
	//stringstream l_out;
	//l_out << "GetQ\n";
	//loop through the tilings
	for(tiling = 0; tiling < m_numTilings; ++tiling)
	{
		//if the tile can exist, check if we have already encountered it
		if(state_keys[tiling].empty() == false)
		{
			pair<string,int> state_action = make_pair(state_keys[tiling],action);
			map<pair<string,int>,double>::iterator it = m_tilings.find(state_action);
			if(it != m_tilings.end()) //If the state-action pair is already in the map
			{
				//l_out << "Found state key " << it->first.first << it->first.second  << " in tiling " << tiling <<". Value: "<< it->second << endl;
				Q_value += it->second; //add found value to Q-value
			} else {				//else
				//l_out << "New key: "  << state_action.first << " " << state_action.second << " in tiling "<< tiling;
				m_tilings[state_action] = DEFAULT_Q; //Set the value to default
				//l_out << ". Value: " << DEFAULT_Q << endl;
				Q_value += m_tilings[state_action]; //Add the value of the newly created entry to the Q-value
			}
		} else {
			cerr << "ERROR: Trying to compute the Q-value of a state that is not in all tilings!\n";
		}
	}
	//l_out << "SUM = " << Q_value << endl;
	//l_out << "End of GetQ\n";
	//mp_log->write(l_out.str());
	return Q_value;
}

//Get the max element of a vector of doubles
double TileCodingHM::myTCMax(const vector<double>& action_values)
{
	double max_val = action_values[0];
	unsigned int idx;
	for(idx = 1; idx < action_values.size(); idx++)
		if(action_values[idx] > max_val)
			max_val = action_values[idx];

	return max_val;
}

//Get the index of the max element of a vector of doubles
int TileCodingHM::myTCArgMax(const vector<double>& action_values)
{
	double max_val = action_values[0];
	int max_idx = 0;
	unsigned int idx;
	for(idx = 1; idx < action_values.size(); idx++)
	{
		if(action_values[idx] > max_val)
		{
			max_val = action_values[idx];
			max_idx = idx;
		}
	}

	return max_idx;
}

//Get the action with the highest Q-value in a given state and put it in a given Action object
void TileCodingHM::getMaxAction(State* state, Action* action)
{
	stringstream ss;
	if(m_verbose)
		ss << "GetMaxAction: \n";
	vector<double> Q_values = vector<double> (numberOfActions, 0.0);

	//For each tile: convert continuous state into discrete states defined by tiles
	vector<string> state_keys;
	for(int tile = 0; tile < m_numTilings; tile++)
	{
		state_keys.push_back(classifyState(state, tile));
	}

	for(int act = 0; act < numberOfActions; ++act)
	{
		Q_values[act] += getQOfStateActionPair(state_keys,act);
	}

	if(m_verbose)
	{
		ss << "Q-values of actions:\n";
		for(unsigned int idx=0; idx < Q_values.size(); ++idx)
			ss << "Action "<< idx << ": " << Q_values[idx] << endl;
	}
	//Get the action with the highest Q-value
	action->discreteAction = myTCArgMax(Q_values); //This is where the action is stored, so no need to return anything
	cout << "TC - Taking Max Action: " << action->discreteAction << endl;
	if(m_verbose)
	{
		ss << "Best Action: " << action->discreteAction << ". \n";
		ss << "End of GetMax Action.";
		mp_log->write(ss.str());
	}
}


//boltzmann implements boltzmann exploration for this TileCoding implementation
void TileCodingHM::boltzmann(State* state, Action * action, double tau ) {
    //Log
	stringstream ss;
	if(m_verbose)
		ss << "Boltzmann: \n";

	////Get the necessary information for computing the exponent
	//Classify the state in tilings
	vector<string> state_keys;
	for(int tiling = 0; tiling < m_numTilings; tiling++)
	{
		state_keys.push_back(classifyState(state,tiling));
	}

	//Get the Q value of each action
	vector<double> q_values;
	int a;
	for(a = 0; a < numberOfActions; a++)
	{
		q_values.push_back(getQOfStateActionPair(state_keys,a));
		if(m_verbose)
			ss << "Q value[" << a << "] = " << q_values[a] << endl;
	}

	//Get the max Q value
    double max_Q = myTCMax(q_values);

	//Just for the log:
	double max_action = myTCArgMax(q_values);
	cout << "max action : " << max_action << endl;
	if(m_verbose)
		ss << "Max action: " << max_action << endl;

	////Do Boltzmann exploration
	//Compute Boltzmann value
	double sum_Qs = 0;
	vector<double> policy;
    for(a = 0; a < numberOfActions; a++)
	{
		policy.push_back( exp( (q_values[a] - max_Q) / tau));
        sum_Qs += policy[a] ;
    }

    for(a = 0; a < numberOfActions; a++)
	{
        policy[a] /= sum_Qs ;
		if(m_verbose)
			ss << "Boltzmann value of action " << a << " : " << policy[a] << endl;
	}

	//Get random number
   	#ifdef WIN32
		double rnd = double(rand())/RAND_MAX;
	#else
		double rnd = drand48();
	#endif
	if(m_verbose)
		ss << "random nr: " << rnd << endl;

	//Choose action
    double total = policy[0];
	a = 0;

    while (a < (numberOfActions - 1) && total < rnd)
	{
        a++;
		total += policy[a];
    }

	if(a <= numberOfActions)
	{
		action->discreteAction = a;
		cout << "Chosen action: " << a << endl;
		if(m_verbose)
			ss << "Chosen action: " << a << endl;
		mp_log->write(ss.str());
		//return; // not necessary, but looks nice.
    } else
	{
		// Something went wrong...
		cerr << "Error in boltzmann:\n";
        cerr << total << " is smaller than " << rnd << endl ;
		cerr << "Boltzmann values: \n";
		int act;
        for (act = 0; act < numberOfActions ; act++)
            cerr << policy[act] << " " ;
        cerr << endl ;
		cerr << "Q values: \n";
        for (act = 0; act < numberOfActions; act++)
            cerr << q_values[act] << " " ;
        cerr << endl ;
		#ifdef WIN32
			char end;
			cin>>end;
		#endif
        exit(-1) ;
	}
}

//writeQTable writes only the Q-values to a file, not the edges of the tiles!!
void TileCodingHM::writeQTable(string filename)
{
	ofstream f_out;
	f_out.open(filename);
	if(f_out.is_open())
	{
		//First, write the parameters.
		f_out	<< m_numTilings << " " << m_tileNum_speed << " " << m_tileNum_pos	<< " " << m_tileNum_angle << " " << m_tileNum_dist << " " << numberOfActions << "\n";

		//Write extra information
		map<pair<string,int>,double>::iterator it;
		f_out << "Total number of entries: " << (int) m_tilings.size() << endl;
		
		//Then write the table
		for(it = m_tilings.begin(); it != m_tilings.end(); ++it)
		{
			//Looks nice, but difficult to load:
			//f_out << "(" << it->first.first << "," << it->first.second << ")" <<" : " << it->second << endl;

			//Less pretty, but easier to load: 
			f_out << it->first.first << " , " << it->first.second << " , " << it->second << endl;
		}

		f_out.close();
	} else {
		cerr << "Could not open file for writing QTable. Please check filename.\n";
	}
}

//loadQTable loads only the Q-values to a file, not the edges of the tiles!!
void TileCodingHM::loadQTable(string filename)
{
	ifstream f_in;
	f_in.open(filename);
	if(f_in.is_open())
	{
		string line;
		//First, load the parameters.
		//NOTE: This will be overwritten by init()
		getline(f_in, line); //skip loading the parameters by doing nothing with this line
		//f_in >> m_numTilings;
		//f_in >> m_tileNum_speed;
		//f_in >> m_tileNum_pos;
		//f_in >> m_tileNum_angle;
		//f_in >> m_tileNum_dist;
		//f_in >> numberOfActions;

		//Next, load the extra information
		getline(f_in, line); //skip loading the extra information by doing nothing with this line

		//Load the Qtable (tilings) per line
		while(getline(f_in,line))
		{
			stringstream ss(line);
			string element;
			int count = 0;
			string state;
			int action;
			double value;

			//Get the three values from the line
			while(ss >> element) //get each element
			{
				if(element.compare(",") == 0)
					continue;

				stringstream number(element);
				switch(count)
				{
					case 0:
						state = element;
						break;
					case 1:
						number >> action;
						break;
					case 2:
						number >> value;
						break;
				}
				++count;
			}

			//Since we have all the information from the line now, we can add the element to m_tilings (Q table)
			pair<string,int> state_action = make_pair(state,action);
			m_tilings[state_action] = value;
			//cout << state << ", " << action << ": "<< value << endl;
		}
		
		f_in.close();
		cout << "Loading QTable is done. Press any key to continue.\n"; // Are you sure that the code-defined edges are the same??
		char a;
		cin >> a;
	} else {
		cerr << "Could not open file for loading the QTable. Please check filename.\n";
	}
}

//Keeps track of how often each state is visited
void TileCodingHM::storeStateVisit(const pair<string,int>& key)
{
	//if state_visits contains the key, add 1 to the value
	if(state_visits.count(key) > 0)
	{
		state_visits[key]++;
		//cout << "Visited state " << key << " =  " << state_visits[key] << endl;
	} else { //else add this state to state_visits and set value to 1
		state_visits[key] = 1;
		//cout << "New state " << key << " =  " << state_visits[key] <<endl;
	}
}

//Writes visited states and their visit counts to a file

void TileCodingHM::writeStateVisits(const string& filename)
{
	ofstream f_out;
	f_out.open(filename);
	if(f_out.is_open())
	{
		map<pair<string,int>,int>::iterator it;
		//f_out << "Total number of visited states: " << (int) state_visits.size() << endl;
		//f_out << "NOTE: This is independent of actions.\n";
		int sum = 0;
		for(it = state_visits.begin(); it != state_visits.end(); ++it)
		{
			f_out << "state: " << it->first.first << ", " << it->first.second << ". Number of visits: " << it->second << endl;
			sum += it->second;
		}
		f_out << "Total number of actions: " << sum / m_numTilings;
		f_out.close();
	} else {
		cerr << "ERROR: Could not open '" << filename << "' for writing state visits.\n";
	}
}

//Keeps track of the average td error in each state action pair
void TileCodingHM::storeAverageTDError(const pair<string,int>& key, double td_error)
{
	if(state_visits.count(key) > 0)
	{
		double curr_td = average_td_errors[key];
		double td_sum = curr_td * (state_visits[key] -1); //we know that state_visits is updated before average_td_errors
		td_sum += td_error;
		average_td_errors[key] = td_sum / state_visits[key];// new average = ((avg * N) + new_val) / (N+1)

	} else { //else add this state to average_td_errors and set value to td_error
		average_td_errors[key] = td_error;
	}
}

//Writes visited states and their visit counts to a file

void TileCodingHM::writeAverageTDError(const string& filename)
{
	ofstream f_out;
	f_out.open(filename);
	if(f_out.is_open())
	{
		map<pair<string,int>,double>::iterator it;
		//f_out << "Total number of visited states: " << (int) state_visits.size() << endl;
		//f_out << "NOTE: This is independent of actions.\n";
		int sum = 0;
		for(it = average_td_errors.begin(); it != average_td_errors.end(); ++it)
		{
			f_out << "State: " << it->first.first << ", " << it->first.second << ". Average TD Error: " << it->second << endl;
			sum += it->second;
		}
		f_out.close();
	} else {
		cerr << "ERROR: Could not open '" << filename << "' for writing TD errors.\n";
	}
}