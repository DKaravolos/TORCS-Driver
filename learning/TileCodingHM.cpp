# include "TileCodingHM.h"

using namespace std ;

TileCodingHM::TileCodingHM(World * w)
{
	init(w);    
}

TileCodingHM::TileCodingHM(World * w, const char* qtable_file)
{
	loadQTable(string(qtable_file));
	init(w);
}

TileCodingHM::~TileCodingHM()
{
	//write information about the visited states
	//cout << "writing state visits to file\n";
	//writeStateVisits("loge_files/state_visits.txt");
}

//initialisation
void TileCodingHM::init(World* w)
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

	//Get parameter/config info from user ??

	//Set number of tilings
	cout << "How many tilings are defined in the edge configuration file?\n";
	cin >> m_numTilings; //NOTE: Vector subscript out of range error is produced when the actual number is larger than this.

	//Set edges of squares in tiles
	setEdges();
}

//Sets the edges of the bins of the tiles using file input
void TileCodingHM::setEdges()
{
	//init sizes, so we can use element access of vectors 
	m_speed_edges.resize(m_numTilings);
	m_trackPos_edges.resize(m_numTilings);
	m_angle_edges.resize(m_numTilings);
	m_dist_edges.resize(m_numTilings);

	getEdgesFromFile("TileCodingEdges.txt"); // assumes to find a maximum of m_numTilings values for tilings
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
					cout << "This is not equal to " << m_numTilings << ". Continue? (y/n)\n";
					char ans;
					cin >> ans;
					if(ans == 'n')
						exit(0);
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

	int current_action = action->discreteAction ;

	//For each tile: convert continuous state into discrete states defined by tiles
	double active_tilings = 0;
	int tiling;
	for(tiling = 0; tiling < m_numTilings; tiling++)
	{
		//Identify the active tiles of this state
		m_state_keys.push_back(classifyState(state, tiling));
		if(m_state_keys[tiling].empty() == false)
			active_tilings++;
		else
			cerr << "ERROR: There is an inactive tiling.\n";

		//Store the visit to this state
		storeStateVisit(m_state_keys[tiling]); 
	}

	//Define the learning rate of this update. This depends on the number of active tilings.
	double learning_rate = learningRate[0] / active_tilings;

	//For each tile: convert continuous next_state into discrete next_states defined by tiles
	for(int tile = 0; tile < m_numTilings; tile++)
	{
		m_next_state_keys.push_back(classifyState(nextState, tile));
	}

	double q_of_state = getQOfStateActionPair(m_state_keys, current_action);
	double td_error;

	if(endOfEpisode)
	{
		//Compute TD error (independent of tiling)
		td_error = rt - q_of_state;

		////Update the Q-values for each active tile (per tiling) depending on the reward
		//for each theta: theta[i][a] = theta[i][a] + lr * (rt- phi[i][a] * theta[i][a]) * phi[i][a]
		for(int tiling = 0; tiling < m_numTilings; tiling++)
		{
			if(m_state_keys[tiling].empty() == false) //no update if the tiling has no active feature (this filters the second phi[i][a] if phi[i][a] = 0 )
			{
				//Only update the active feature in a tiling. (this filters the first phi[i][a] if phi[i][a] = 0)
				pair<string,int> state_action = make_pair (m_state_keys[tiling], current_action);
				m_tilings[state_action]	+= learning_rate * td_error;
			}
		}
	} else
	{
		//Compute max action of next state (independent of tiling)
		double max_q = getMaxQOfState(m_next_state_keys);

		//Compute TD error (independent of tiling)
		td_error = (rt + gamma * max_q) - q_of_state;

		////Update the Q-values for each active tile (per tiling) depending on the reward and the next state
		//for each theta: theta[i][a] = theta[i][a] + lr * (rt + gmma * maxQs - phi[i][a] * theta[i][a]) * phi[i][a]
		//// note: i = tiling, a = at (=action)
		for(int tiling = 0; tiling < m_numTilings; tiling++)
		{
			if(m_state_keys[tiling].empty() == false && m_next_state_keys[tiling].empty() == false) //no update if the tiling has no active feature (this filters the second phi[i][a] if phi[i][a] = 0 )
			{
				//Only update the active feature in a tiling. (this filters the first phi[i][a] if phi[i][a] = 0)
				pair<string,int> state_action = make_pair (m_state_keys[tiling], current_action);
				m_tilings[state_action]	+= learning_rate * td_error;
			} else {
				cerr << "This state is not classified by tiling " << tiling << ", update will be partial (and thus wrong).\n"; 
			}
		}		
	}

	return td_error;
}

// Find the tile indices for a given state. Returns NULL if the state is not in this tile.
string TileCodingHM::classifyState(const State* state, const int& tiling)
{
	stringstream key;
	int tile_indices[8]; //NOTE: It should be possible to use a vector<int>* for this
	//Classify each dimension separately
	tile_indices[0] = classifyValue(state->continuousState[0], m_speed_edges[tiling]);
	tile_indices[1] = classifyValue(state->continuousState[1], m_trackPos_edges[tiling]);
	tile_indices[2] = classifyValue(state->continuousState[2], m_angle_edges[tiling]);
	tile_indices[3] = classifyValue(state->continuousState[3], m_dist_edges[tiling]);
	tile_indices[4] = classifyValue(state->continuousState[4], m_dist_edges[tiling]);
	tile_indices[5] = classifyValue(state->continuousState[5], m_dist_edges[tiling]);
	tile_indices[6] = classifyValue(state->continuousState[6], m_dist_edges[tiling]);
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
		} else {
			cerr << "Oh no! Dimension " << idx << " does not fit in a tile of tiling " << tiling << endl;
			return ""; //If any index is out of bounds, the state is not in this tiling -> quit creating key and create error 			
		}
	}

	return key.str();
}

//Classifies a given value into the given tile bins
int TileCodingHM::classifyValue(const double& state_value, const vector<double>& bin_edges)
{
	int bin = -1; 
	for(int idx = 0; idx < bin_edges.size()-1; idx++)
		if(state_value >= bin_edges[idx] && state_value < bin_edges[idx+1]) //check each bin
		{
			bin = idx;
			break;
		}

	return bin; //If the state_value is outside the tile range, it returns -1. 
				//This is sort of an exception code, it should be caught later, because indices cannot be -1
}

//Gets the max action-value of a state (vector of strings)
double TileCodingHM::getMaxQOfState(const vector<string>& state_keys)
{
	vector<double> Q_values = vector<double> (numberOfActions, 0.0);

	//Find the Q-value of all actions given the tiles
	for(int tiling = 0; tiling < m_numTilings; ++tiling)
	{
		for(int act = 0; act < numberOfActions; ++act)
		{
			if(state_keys[tiling].empty() == false)
			{
				pair<string,int> state_action = make_pair(state_keys[tiling],act);
				Q_values[act] += m_tilings[state_action];
			}
		}
	}

	return myTCMax(Q_values);
}

//Gets the Q-value of a specific tile
double TileCodingHM::getQOfStateActionPair(const vector<string>& state_keys, int action)
{
	double Q_value = 0;
	int tiling;
	//loop through the tilings
	for(tiling = 0; tiling < m_numTilings; ++tiling)
	{
		//if the tile exists, add the value to the sum
		if(state_keys[tiling].empty() == false)
		{
			pair<string,int> state_action = make_pair(state_keys[tiling],action);
			Q_value += m_tilings[state_action];
		} else {
			cerr << "ERROR: Trying to compute the Q-value of a tile that is not in all tilings!\n";
		}
	}
	return Q_value;
}

//Get the max element of a vector of doubles
double TileCodingHM::myTCMax(const vector<double>& action_values)
{
	double max_val = action_values[0];
	int idx;
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
	int idx;
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
	vector<double> Q_values = vector<double> (numberOfActions, 0.0);

	//For each tile: convert continuous state into discrete states defined by tiles
	vector<string> state_keys;
	for(int tile = 0; tile < m_numTilings; tile++)
	{
		state_keys.push_back(classifyState(state, tile));
	}

	//Find the Q-value of all actions given the tiles
	for(int tiling = 0; tiling < m_numTilings; ++tiling)
	{
		for(int act = 0; act < numberOfActions; ++act)
		{
			if(state_keys[tiling].empty() == false)
			{
				pair<string,int> state_action = make_pair(state_keys[tiling],act);
				Q_values[act] += m_tilings[state_action];
			}
		}
	}

	//Get the action with the highest Q-value
	action->discreteAction = myTCArgMax(Q_values); //This is where the action is stored, so no need to return anything
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
		cout << "Loading QTable is done. Are you sure that the code-defined edges are the same??\n";
		char a;
		cin >> a;
	} else {
		cerr << "Could not open file for loading the QTable. Please check filename.\n";
	}
}

//Keeps track of how often each state is visited
void TileCodingHM::storeStateVisit(const string& key)
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
		map<string,int>::iterator it;
		f_out << "Total number of visited states: " << (int) state_visits.size() << endl;
		f_out << "NOTE: This is independent of actions.\n";
		for(it = state_visits.begin(); it != state_visits.end(); ++it)
		{
			f_out << "state: " << it->first << ". Number of visits: " << it->second << endl;
		}

		f_out.close();
	} else {
		cerr << "ERROR: Could not open '" << filename << "' for writing state visits.\n";
	}
}