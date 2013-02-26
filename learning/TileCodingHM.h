#ifndef TILECODINGHM_H
#define TILECODINGHM_H
#include <stdlib.h>
# include <iostream>
# include <sstream>
# include <string>
# include <vector>
# include <deque>
# include <map>
# include "../rlcpp/StateActionAlgorithm.h"
# include "../rlcpp/State.h"
# include "../utilities/Writer.h"
# include "../utilities/translateAction.h"

using namespace std;

class TileCodingHM : public StateActionAlgorithm {
    public:
        TileCodingHM(World * w, const string& log_dir);
		TileCodingHM(World * w, const string& log_dir, const char* qtable_file);
        ~TileCodingHM();

        void update	(State * st, Action * action, double rt, State * st_,
					bool endOfEpisode, double * learningRate, double gamma);

		double updateAndReturnTDError	(State * st, Action * action, double rt, State * st_,
										bool endOfEpisode, double * learningRate, double gamma);
		
		void getMaxAction(State* state, Action* action);
		int getStateCount(const string& state);
		bool isStateKnown(const State& state);
		void loadQTable (string filename);
		void writeQTable (string filename);
		void writeStateVisits(const string& filename);
		void writeAverageTDError(const string& filename);
		void writeStateInfo(const string& filename);
		double getQOfStateActionPair(const vector<string>& state_bins, int action); //dit is voor Writer

		inline unsigned int getNumberOfLearningRates()	{return 1;}
		inline const char * getName()					{return "TileCoding" ;}
		
	protected:
		void boltzmann (State* s, Action* a, double tau);

	private:
		//Log
		Writer* mp_log;
		Writer* mp_td_log;
		bool m_verbose;

		//QTable:
		//tile,	speed,	trackpos,angle,		dist1, dist2, dist3, dist4, dist5,		actions
		map<pair<string,int>,double> m_tilings;

		//Container for indices of mp_tiles
		vector<string> m_state_keys;
		vector<string> m_next_state_keys;

		//tile info
		int m_numTilings;
		int m_tileNum_speed;
		int m_tileNum_pos;
		int m_tileNum_angle;
		int m_tileNum_dist;

		//edges of squares in tiles
		vector<vector<double> > m_speed_edges;
		vector<vector<double> > m_trackPos_edges;
		vector<vector<double> > m_angle_edges;
		vector<vector<double> > m_dist_edges;

		// init functions
		//void getTileCodingSettings(string parameterfile);
		void init(World* world, const string& log_dir);
		void setEdges();
		void getEdgesFromFile(string filename);
		void addEdge(string& type, stringstream& ss, vector<vector<double> >& edge_vector, int& tiling);


		//Q-value functions
		double getMaxQOfState(const vector<string>& state);
		double getMaxActionOfState(const vector<string>& state);
		//double getQOfStateActionPair(const vector<string>& state_bins, int action);
		double myTCMax(const vector<double>& action_values);
		int myTCArgMax(const vector<double>& action_values);

		//Classification of input values (to tile indices)
		string classifyState(const State* state, const int& tile);
		int classifyValue(const double& state_value, const vector<double>& bin_edges);
		int classifyValue(const double& state_value, const vector<double>& bin_edges, bool verbose);

		// Keep track of how often each state is visited
		map<pair<string,int>,int> state_visits;
		map<pair<string,int>,double> average_td_errors;
		void storeStateActionVisit(const pair<string,int>& key);
		void storeAverageTDError(const pair<string,int>& key, double td_error);

		// compute td error over all states
		void computeGeneralTDError(double td_error);
		int m_nr_of_updates;
		double m_avg_td_error;
		//Analyse TD error
		void checkTDError(const double& td_error, const double& q_of_state, const int& action, const double& reward, const double& q_next);

};

#endif //TILECODINGHM_H
