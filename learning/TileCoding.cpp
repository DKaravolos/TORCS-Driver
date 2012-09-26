# include "TileCoding.h"

using namespace std ;

TileCoding::TileCoding(World * w)
{
	init(w);    
}

TileCoding::TileCoding(World * w, const char* qtable_file)
{
	init(w);
	loadQTable(string(qtable_file));
}

TileCoding::~TileCoding()
{
	//write information about the visited states
	//cout << "writing state visits to file\n";
	//writeStateVisits("loge_files/state_visits.txt");
}

//initialisation
void TileCoding::init(World* w)
{
	discreteStates = w->getDiscreteStates() ;

    if (!w->getDiscreteActions() || w->getDiscreteStates()) {

        cout << "This TileCoding does not support continuous actions or discrete states." << endl ;
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

	//Get tile info from user
	//string l_filename;
	//cout << "Where can I find the tile coding parameters?\n";
	//cin >> l_filename;
	//getTileCodingSettings(l_filename); //Gebruik pas als tile coding geïmplementeerd is.

	//Set number of tiles 
	m_numTiles = 2;

	//Set tile size
	m_tileNum_speed = 5;
	m_tileNum_pos = m_tileNum_angle = m_tileNum_dist = 5;
	//m_tileNum_angle = 3;
	//m_tileNum_dist = 3;


	//Set tiles to initial value
	//Let's try optimistic initialisation to facilitate exploration
	//maximal reward is 15?
	m_tilings = vector<	vector<	vector<	vector<	vector<vector<vector<vector<vector<	vector<double>>>>>>>>>> (m_numTiles,
						vector<	vector<	vector<	vector<vector<vector<vector<vector<	vector<double>>>>>>>>>	(m_tileNum_speed,
								vector<	vector<	vector<vector<vector<vector<vector<	vector<double>>>>>>>>	(m_tileNum_pos,
										vector<	vector<vector<vector<vector<vector<	vector<double>>>>>>>	(m_tileNum_angle,
												vector<vector<vector<vector<vector<	vector<double>>>>>>		(m_tileNum_dist,
														vector<vector<vector<vector<vector<double>>>>> (m_tileNum_dist,
																vector<vector<vector<vector<double>>>> (m_tileNum_dist,
																		vector<vector<vector<double>>> (m_tileNum_dist,
																				vector<vector<double>> (m_tileNum_dist,
																						vector<double> (numberOfActions, 0.0))))))))));
	
	//Set edges of squares in tiles
	//MAKE SURE THAT ALL POSSIBLE STATE VALUES FIT IN ALL TILES AT THE SAME TIME!
	setEdges();
}

//Sets the edges of the bins of the tiles using file input
void TileCoding::setEdges()
{
	//init sizes, so we can use element access of vectors 
	m_speed_edges.resize(m_numTiles);
	m_trackPos_edges.resize(m_numTiles);
	m_angle_edges.resize(m_numTiles);
	m_dist_edges.resize(m_numTiles);

	for (int i = 0; i < m_numTiles; ++i)
	{
		m_speed_edges[i].resize(m_tileNum_speed+1);
		m_trackPos_edges[i].resize(m_tileNum_pos+1);
		m_angle_edges[i].resize(m_tileNum_angle+1);
		m_dist_edges[i].resize(m_tileNum_dist+1);
	}

	getEdgesFromFile("TileCodingEdges.txt"); // assumes to find m_numTiles values for tilings
}

//Reads the edges of the bins of the tiles from a file
void TileCoding::getEdgesFromFile(string l_parameterfile)
{
	ifstream is;
	is.open(l_parameterfile);
	if(is.is_open())
	{
		try
		{
			int t;
			//NOTE: WE ASSUME THAT THERE ARE m_numTiles DEFINED IN THE CONFIGURATION FILE!!
			for(t = 0; t < m_numTiles; t++)
			{
					//TILING t
					//Get speed edges tiling t
					is >> m_speed_edges[t][0];
					is >> m_speed_edges[t][1];
					is >> m_speed_edges[t][2];
					is >> m_speed_edges[t][3];
					is >> m_speed_edges[t][4];
					is >> m_speed_edges[t][5];

					//Get other edges tiling t
					is >> m_trackPos_edges[t][0];
					is >> m_trackPos_edges[t][1];
					is >> m_trackPos_edges[t][2];
					is >> m_trackPos_edges[t][3];
					is >> m_trackPos_edges[t][4];
					is >> m_trackPos_edges[t][5];

					m_angle_edges[t][0] = m_dist_edges[t][0] = m_trackPos_edges[t][0];
					m_angle_edges[t][1] = m_dist_edges[t][1] = m_trackPos_edges[t][1];
					m_angle_edges[t][2] = m_dist_edges[t][2] = m_trackPos_edges[t][2];
					m_angle_edges[t][3] = m_dist_edges[t][3] = m_trackPos_edges[t][3];
					m_angle_edges[t][4] = m_dist_edges[t][4] = m_trackPos_edges[t][4];
					m_angle_edges[t][5] = m_dist_edges[t][5] = m_trackPos_edges[t][5];
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

//same as updateAndReturnTDError, but does not return TD error
void TileCoding::update( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma)
{
	updateAndReturnTDError(state,action,rt,nextState,endOfEpisode,learningRate,gamma);
}

//Performs an update of the Q-table by computing the TD error explicitly and returns the TD error.
double TileCoding::updateAndReturnTDError( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma)
{
	if (!state->continuous )
	{
		cerr << "Tile coding does not support discrete states!! Can't update!\n";
		return 0;
	}

	int at = action->discreteAction ;

	//For each tile: convert continuous state into discrete states defined by tiles
	double active_tilings = 0;
	int tiling;
	for(tiling = 0; tiling < m_numTiles; tiling++)
	{
		m_state_bins.push_back(classifyState(state, tiling));
		if(m_state_bins[tiling] != NULL)
			active_tilings++;
		storeStateVisit(m_state_bins[tiling]);
	}

	//Store the visit to this state
		

	//Define the learning rate of this update. This depends on the number of active tilings.
	double learning_rate = learningRate[0] / active_tilings;

	//For each tile: convert continuous next_state into discrete next_states defined by tiles
	for(int tile = 0; tile < m_numTiles; tile++)
	{
		m_next_state_bins.push_back(classifyState(nextState, tile));
	}

	double q_of_state = getQOfStateActionPair(m_state_bins, at);
	double td_error;

	if(endOfEpisode)
	{
		//Compute TD error (independent of tiling)
		td_error = rt - q_of_state;

		////Update the Q-values for each active tile (per tiling) depending on the reward
		//for each theta: theta[i][a] = theta[i][a] + lr * (rt- phi[i][a] * theta[i][a]) * phi[i][a]
		for(int tiling = 0; tiling < m_numTiles; tiling++)
		{
			if(m_state_bins[tiling] != NULL) //no update if the tiling has no active feature (this filters the second phi[i][a] if phi[i][a] = 0 )
			{
				m_tilings[tiling]  //Only update the active feature in a tiling. (this filters the first phi[i][a] if phi[i][a] = 0)
					[m_state_bins[tiling][0]][m_state_bins[tiling][1]]	
					[m_state_bins[tiling][2]][m_state_bins[tiling][3]]
					[m_state_bins[tiling][4]][m_state_bins[tiling][5]]
					[m_state_bins[tiling][6]][m_state_bins[tiling][7]]
					[at] 		
				+= learning_rate * td_error;
			}
		}
	} else
	{
		//Compute max action of next state (independent of tiling)
		double max_q = getMaxQOfState(m_next_state_bins);

		//Compute TD error (independent of tiling)
		td_error = (rt + gamma * max_q) - q_of_state;

		////Update the Q-values for each active tile (per tiling) depending on the reward and the next state
		//for each theta: theta[i][a] = theta[i][a] + lr * (rt + gmma * maxQs - phi[i][a] * theta[i][a]) * phi[i][a]
		//// note: i = tiling, a = at (=action)
		for(int tiling = 0; tiling < m_numTiles; tiling++)
		{
			if(m_state_bins[tiling] != NULL && m_next_state_bins[tiling] != NULL) //no update if the tiling has no active feature (this filters the second phi[i][a] if phi[i][a] = 0 )
			{
				m_tilings[tiling]	//Only update the active feature in a tiling. (this filters the first phi[i][a] if phi[i][a] = 0)
					[m_state_bins[tiling][0]][m_state_bins[tiling][1]]
					[m_state_bins[tiling][2]][m_state_bins[tiling][3]]
					[m_state_bins[tiling][4]][m_state_bins[tiling][5]]
					[m_state_bins[tiling][6]][m_state_bins[tiling][7]]
					[at] 
				+= learning_rate * td_error;
			} else {
				cerr << "This state is not classified by tiling " << tiling << ", update will be partial (and thus wrong).\n"; 
			}
		}		
	}

	//clean up allocated memory
	for(int tile = 0; tile < m_numTiles; tile++)
	{
		delete m_state_bins[tile];
		delete m_next_state_bins[tile]; 
	}
	m_state_bins.clear();
	m_next_state_bins.clear();

	return td_error;
}

// Find the tile indices for a given state. Returns NULL if the state is not in this tile.
int* TileCoding::classifyState(const State* state, const int& tiling)
{
	int* tile_indices = new int[8]; //NOTE: It should be possible to use a vector<int>* for this
	//Classify each dimension separately
	tile_indices[0] = classifyValue(state->continuousState[0], m_speed_edges[tiling]);
	tile_indices[1] = classifyValue(state->continuousState[1], m_trackPos_edges[tiling]);
	tile_indices[2] = classifyValue(state->continuousState[2], m_angle_edges[tiling]);
	tile_indices[3] = classifyValue(state->continuousState[3], m_dist_edges[tiling]);
	tile_indices[4] = classifyValue(state->continuousState[4], m_dist_edges[tiling]);
	tile_indices[5] = classifyValue(state->continuousState[5], m_dist_edges[tiling]);
	tile_indices[6] = classifyValue(state->continuousState[6], m_dist_edges[tiling]);
	tile_indices[7] = classifyValue(state->continuousState[7], m_dist_edges[tiling]);

	//Check if there was any value out of bounds
	int idx;
	for(idx = 0; idx < 8; ++idx)
	{
		if(tile_indices[idx] < 0)
		{
			delete[] tile_indices;
			cerr << "Oh no! Dimension " << idx << " does not fit in a tile of tiling " << tiling << endl;
			return NULL; //If any index is out of bounds, the state is not in this tiling. 
		}
	}

	return tile_indices;
}

//Classifies a given value into the given tile bins
int TileCoding::classifyValue(const double& state_value, const vector<double>& bin_edges)
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

//Get the max element of a state (vector of int pointers)
double TileCoding::getMaxQOfState(const vector<int*>& state_bins)
{
	vector<double> Q_values = vector<double> (numberOfActions, 0.0);

	//Find the Q-value of all actions given the tiles
	for(int tile = 0; tile < m_numTiles; ++tile)
	{
		for(int act = 0; act < numberOfActions; ++act)
		{
			if(state_bins[tile] != NULL)
			{
				Q_values[act] += m_tilings[tile]
						[state_bins[tile][0]][state_bins[tile][1]]
						[state_bins[tile][2]][state_bins[tile][3]]
						[state_bins[tile][4]][state_bins[tile][5]]
						[state_bins[tile][6]][state_bins[tile][7]]
						[act];
			}
		}
	}

	return myTCMax(Q_values);
}

//Gets the Q-value of a specific tile
double TileCoding::getQOfStateActionPair(const vector<int*>& state_bins, int action)
{
	double Q_value = 0;
	int tiling;
	//loop through the tilings
	for(tiling = 0; tiling < m_numTiles; ++tiling)
	{
		//if the tile exists, add the value to the sum
		if(state_bins[tiling] != NULL)
		{
			Q_value += m_tilings[tiling]
					[state_bins[tiling][0]][state_bins[tiling][1]]
					[state_bins[tiling][2]][state_bins[tiling][3]]
					[state_bins[tiling][4]][state_bins[tiling][5]]
					[state_bins[tiling][6]][state_bins[tiling][7]]
					[action];
		} else {
			cerr << "ERROR: Trying to compute the Q-value of a tile that is not in all tilings!\n";
		}
	}
	return Q_value;
}

//Get the max element of a vector of doubles
double TileCoding::myTCMax(const vector<double>& action_values)
{
	double max_val = action_values[0];
	int idx;
	for(idx = 1; idx < action_values.size(); idx++)
		if(action_values[idx] > max_val)
			max_val = action_values[idx];

	return max_val;
}

//Get the index of the max element of a vector of doubles
int TileCoding::myTCArgMax(const vector<double>& action_values)
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
void TileCoding::getMaxAction(State* state, Action* action)
{
	vector<double> Q_values = vector<double> (numberOfActions, 0.0);

	//For each tile: convert continuous state into discrete states defined by tiles
	vector<int*> state_bins;
	for(int tile = 0; tile < m_numTiles; tile++)
	{
		state_bins.push_back(classifyState(state, tile));
	}

	//Find the Q-value of all actions given the tiles
	for(int tile = 0; tile < m_numTiles; ++tile)
	{
		for(int act = 0; act < numberOfActions; ++act)
		{
			if(state_bins[tile] != NULL)
			{
				Q_values[act] += m_tilings[tile]
						[state_bins[tile][0]][state_bins[tile][1]]
						[state_bins[tile][2]][state_bins[tile][3]]
						[state_bins[tile][4]][state_bins[tile][5]]
						[state_bins[tile][6]][state_bins[tile][7]]
						[act];
			}
		}
	}

	//Get the action with the highest Q-value
	action->discreteAction = myTCArgMax(Q_values); //This is where the action is stored, so no need to return anything

	//Clean up the mess
	for(int tile = 0; tile < m_numTiles; ++tile)
	{
		if(state_bins[tile] != NULL)
			delete[] state_bins[tile];
	}
}

//writeQTable writes only the Q-values to a file, not the edges of the tiles!!
void TileCoding::writeQTable(string filename)
{
	ofstream f_out;
	f_out.open(filename);
	if(f_out.is_open())
	{
		//First, write the parameters.
		f_out	<< m_numTiles << " " << m_tileNum_speed << " " << m_tileNum_pos	<< " " << m_tileNum_angle << " " << m_tileNum_dist << " " << numberOfActions << "\n";


		//Then, write the table.
		int l_whole_tile;
		int l_speed_tile;
		int l_pos_tile;
		int l_angle_tile;
		int l_dist_tile1;
		int l_dist_tile2;
		int l_dist_tile3;
		int l_dist_tile4;
		int l_dist_tile5;
		int action;

		for(l_whole_tile = 0; l_whole_tile < m_numTiles; l_whole_tile++)
		{
			for(l_speed_tile = 0; l_speed_tile < m_tileNum_speed; l_speed_tile++)
			{
				for(l_pos_tile = 0; l_pos_tile < m_tileNum_pos; l_pos_tile++)
				{
					for(l_angle_tile = 0; l_angle_tile < m_tileNum_angle; l_angle_tile++)
					{
						for(l_dist_tile1 = 0; l_dist_tile1 < m_tileNum_dist; l_dist_tile1++)
						{
							for(l_dist_tile2 = 0; l_dist_tile2 < m_tileNum_dist; l_dist_tile2++)
							{
								for(l_dist_tile3 = 0; l_dist_tile3 < m_tileNum_dist; l_dist_tile3++)
								{
									for(l_dist_tile4 = 0; l_dist_tile4 < m_tileNum_dist; l_dist_tile4++)
									{
										for(l_dist_tile5 = 0; l_dist_tile5 < m_tileNum_dist; l_dist_tile5++)
										{
											for(action = 0; action < numberOfActions; action++)
											{
												f_out << m_tilings[l_whole_tile][l_speed_tile][l_pos_tile][l_angle_tile][l_dist_tile1][l_dist_tile2][l_dist_tile3][l_dist_tile4][l_dist_tile5][action];
												f_out << "\t";
											}
										}
									}
								}
							}
						}
					}
				}
			}
			f_out << "\n\n";
		}

		f_out.close();
	} else {
		cerr << "Could not open file for writing QTable. Please check filename.\n";
	}
}

//loadQTable loads only the Q-values to a file, not the edges of the tiles!!
void TileCoding::loadQTable(string filename)
{
	ifstream f_in;
	f_in.open(filename);
	if(f_in.is_open())
	{
		//First, write the parameters.
		f_in >> m_numTiles;
		f_in >> m_tileNum_speed;
		f_in >> m_tileNum_pos;
		f_in >> m_tileNum_angle;
		f_in >> m_tileNum_dist;
		f_in >> numberOfActions;

		//Then write the table.
		int l_whole_tile;
		int l_speed_tile;
		int l_pos_tile;
		int l_angle_tile;
		int l_dist_tile1;
		int l_dist_tile2;
		int l_dist_tile3;
		int l_dist_tile4;
		int l_dist_tile5;
		int action;

		for(l_whole_tile = 0; l_whole_tile < m_numTiles; l_whole_tile++)
			for(l_speed_tile = 0; l_speed_tile < m_tileNum_speed; l_speed_tile++)
				for(l_pos_tile = 0; l_pos_tile < m_tileNum_pos; l_pos_tile++)
					for(l_angle_tile = 0; l_angle_tile < m_tileNum_angle; l_angle_tile++)
						for(l_dist_tile1 = 0; l_dist_tile1 < m_tileNum_dist; l_dist_tile1++)
							for(l_dist_tile2 = 0; l_dist_tile2 < m_tileNum_dist; l_dist_tile2++)
								for(l_dist_tile3 = 0; l_dist_tile3 < m_tileNum_dist; l_dist_tile3++)
									for(l_dist_tile4 = 0; l_dist_tile4 < m_tileNum_dist; l_dist_tile4++)
										for(l_dist_tile5 = 0; l_dist_tile5 < m_tileNum_dist; l_dist_tile5++)
											for(action = 0; action < numberOfActions; action++)
												f_in >> m_tilings[l_whole_tile][l_speed_tile][l_pos_tile]
																[l_angle_tile][l_dist_tile1][l_dist_tile2]
																[l_dist_tile3][l_dist_tile4][l_dist_tile5][action];
		f_in.close();
		cout << "Loading QTable is done. Are you sure that the code-defined edges are the same??\n";
		char a;
		cin >> a;
	} else {
		cerr << "Could not open file for loading the QTable. Please check filename.\n";
	}
}

//Keeps track of how often each state is visited
void TileCoding::storeStateVisit(const int* tile_indices)
{
	//create key for the map (dictionary) from tile indices
	stringstream sskey;
	int idx;
	for(idx = 0; idx <= stateDimension; ++idx)
	{
		sskey << tile_indices[idx];
	}
	string key = sskey.str();

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
void TileCoding::writeStateVisits(const string& filename)
{
	ofstream f_out;
	f_out.open(filename);
	if(f_out.is_open())
	{
		map<string,int>::iterator it;
		f_out << "Total number of visited states: " << (int) state_visits.size() << endl;
		for(it = state_visits.begin(); it != state_visits.end(); ++it)
		{
			f_out << "state: " << it->first << ". Number of visits: " << it->second << endl;
		}

		f_out.close();
	} else {
		cerr << "ERROR: Could not open '" << filename << "' for writing state visits.\n";
	}
}