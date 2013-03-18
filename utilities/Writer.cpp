#include "Writer.h"
#define NUM_TILINGS 5 // LET OP!!! HARDCODED HOEVEEL TILINGS ER ZIJN!!
using namespace std;

Writer::Writer(string file_name)
{
	m_file = file_name;

	ofstream out (m_file.c_str(), ios::trunc);
	if(out.is_open()) 
	{
		out << "";
		out.close();
	}
	else{
		cerr << "Can't open file '" << m_file << "'\n";
	}
}

void Writer::write(string message)
{
	//cout << "writing without bool: " << message << endl ;
	ofstream out (m_file.c_str(), ios::app);
	if( out.is_open()) 
	{
		out << message << endl;
		out.close();
	}
	else 
		cout << "Can't open file '" << m_file << "'\n";
}

void Writer::write(string message, bool append)
{
	//cout << "writing with bool: " << message << endl ;
	ofstream out;
	if(append)
		out.open(m_file.c_str(), ios::app);
	else 
		out.open(m_file.c_str(),ios::trunc);

	if( out.is_open()) 
	{
		out << message << endl;
		out.close();
	}
	else 
		cout << "Can't open file '" << m_file << "'\n";
}

void Writer::writeActionTable (const State& cont_state, const std::vector<std::string>& state_keys, map<pair<string,int>, double>& qtable) //, const TileCodingHM* tc_ptr
{
	ofstream out (m_file.c_str(), ios::app);
	if( out.is_open()) 
	{
		out << setfill('-') << setw(60)<<"-"<<endl;
		out << setfill(' ');
		out << "Time: " << cont_state.time_step << endl;
		out << "Speed: " << setprecision(3) << cont_state.continuousState[0];
		out << setw(16) << "Trackpos: " << setprecision(2) << cont_state.continuousState[1];
		out << setw(16) << "Angle: " << setprecision(2) << cont_state.continuousState[2] << endl;
		out << setw(7) << "L " << setw(12) << "L " << setw(12) << "M " << setw(12) << "R " << setw(12) << "R " << endl;
		out	<< setprecision(3)
			<< setw(6) << cont_state.continuousState[3]
			<< setw(12) << cont_state.continuousState[4] 
			<< setw(12) << cont_state.continuousState[5] 
			<< setw(12) << cont_state.continuousState[6] 
			<< setw(12) << cont_state.continuousState[7];
		out << "\n\n";

		out << "Actions values:\n";
		out << setw(6) << "0.5L" << setw(12) << "0.1L" << setw(12) << "M" << setw(12) << "0.1R" << setw(12) << "0.5R" << endl;
		out << setprecision(2) << setw(6)<< getQOfStateActionPair(qtable, state_keys,0) << setw(12)<< getQOfStateActionPair(qtable, state_keys,3) << setw(12)<< getQOfStateActionPair(qtable, state_keys,6) << setw(12)<< getQOfStateActionPair(qtable, state_keys,9)  << setw(12)<< getQOfStateActionPair(qtable, state_keys,12) << endl;
		out << setprecision(2) << setw(6)<< getQOfStateActionPair(qtable, state_keys,1) << setw(12)<< getQOfStateActionPair(qtable, state_keys,4) << setw(12)<< getQOfStateActionPair(qtable, state_keys,7) << setw(12)<< getQOfStateActionPair(qtable, state_keys,10) << setw(12)<< getQOfStateActionPair(qtable, state_keys,13) << endl;
		out << setprecision(2) << setw(6)<< getQOfStateActionPair(qtable, state_keys,2) << setw(12)<< getQOfStateActionPair(qtable, state_keys,5) << setw(12)<< getQOfStateActionPair(qtable, state_keys,8) << setw(12)<< getQOfStateActionPair(qtable, state_keys,11) << setw(12)<< getQOfStateActionPair(qtable, state_keys,14) << endl;
		out.close();
	}
	else 
		cout << "Can't open file '" << m_file << "'\n";
}


//Gets the Q-value of a specific tile
double Writer::getQOfStateActionPair(map<pair<string,int>, double>& qtable, const vector<string>& state_keys, int action)
{
	double Q_value = 0;
	int tiling;
	//loop through the tilings
	for(tiling = 0; tiling < NUM_TILINGS; ++tiling)
	{
		//if the tile can exist, check if we have already encountered it
		if(state_keys[tiling].empty() == false)
		{
			pair<string,int> state_action = make_pair(state_keys[tiling],action);
			map<pair<string,int>,double>::iterator it = qtable.find(state_action);
			if(it != qtable.end()) //If the state-action pair is already in the map
			{
				Q_value += it->second; //add found value to Q-value
			} else {				//else
				cerr<< "ERROR! Trying to print a state that is unknown!";
			}
		} else {
			cerr << "ERROR: Trying to compute the Q-value of a state that is not in all tilings!\n";
		}
	}

	return Q_value;
}

void Writer::writeActionTable (const State& cont_state, const std::vector<double>& q_values) //, const TileCodingHM* tc_ptr
{
	ofstream out (m_file.c_str(), ios::app);
	if(out.is_open()) 
	{
		out << setfill('-') << setw(60)<<"-"<<endl;
		out << setfill(' ');
		out << "Time: " << cont_state.time_step << endl;
		out << "Speed: " << setprecision(3) << cont_state.continuousState[0];
		out << setw(16) << "Trackpos: " << setprecision(2) << cont_state.continuousState[1];
		out << setw(16) << "Angle: " << setprecision(2) << cont_state.continuousState[2] << endl;
		out << setw(7) << "L " << setw(12) << "L " << setw(12) << "M " << setw(12) << "R " << setw(12) << "R " << endl;
		out	<< setprecision(3)
			<< setw(6) << cont_state.continuousState[3]
			<< setw(12) << cont_state.continuousState[4] 
			<< setw(12) << cont_state.continuousState[5] 
			<< setw(12) << cont_state.continuousState[6] 
			<< setw(12) << cont_state.continuousState[7];
		out << "\n\n";

		out << "Actions values:\n";
		out << setw(6) << "0.5L" << setw(12) << "0.1L" << setw(12) << "M" << setw(12) << "0.1R" << setw(12) << "0.5R" << endl;
		out << setprecision(2) << setw(6)<< q_values[0] << setw(12)<< q_values[3] << setw(12)<< q_values[6] << setw(12)<< q_values[9]  << setw(12)<< q_values[12]<< endl;
		out << setprecision(2) << setw(6)<< q_values[1] << setw(12)<< q_values[4] << setw(12)<< q_values[7] << setw(12)<< q_values[10] << setw(12)<< q_values[13] << endl;
		out << setprecision(2) << setw(6)<< q_values[2] << setw(12)<< q_values[5] << setw(12)<< q_values[8] << setw(12)<< q_values[11] << setw(12)<< q_values[14] << endl;
		out.close();
	}
	else 
		cout << "Can't open file '" << m_file << "'\n";
}