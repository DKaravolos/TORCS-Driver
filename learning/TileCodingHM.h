#ifndef TILECODINGHM_H
#define TILECODINGHM_H
# include <iostream>
# include <sstream>
# include <string>
# include <vector>
# include <map>
# include "../rlcpp/StateActionAlgorithm.h"
# include "../rlcpp/State.h"

using namespace std;
class TileCodingHM : public StateActionAlgorithm {
    public:
        TileCodingHM(World * w);
		TileCodingHM(World * w, const char* qtable_file);
        ~TileCodingHM();

        void update	(State * st, Action * action, double rt, State * st_,
					bool endOfEpisode, double * learningRate, double gamma);

		double updateAndReturnTDError	(State * st, Action * action, double rt, State * st_,
										bool endOfEpisode, double * learningRate, double gamma);
		
		void getMaxAction(State* state, Action* action);
		void loadQTable (string filename);
		void writeQTable (string filename);
		void writeStateVisits(const string& filename);

		inline unsigned int getNumberOfLearningRates()	{return 1;}
		inline const char * getName()					{return "TileCoding" ;}
	private:
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
		vector<vector<double>> m_speed_edges;
		vector<vector<double>> m_trackPos_edges;
		vector<vector<double>> m_angle_edges;
		vector<vector<double>> m_dist_edges;

		// init functions
		//void getTileCodingSettings(string parameterfile);
		void init(World* world);
		void setEdges();
		void getEdgesFromFile(string filename);
		void addEdge(string& type, stringstream& ss, vector<vector<double>>& edge_vector, int& tiling);


		//Q-value functions
		double getMaxQOfState(const vector<string>& state);
		double getQOfStateActionPair(const vector<string>& state_bins, int action);
		double myTCMax(const vector<double>& action_values);
		int myTCArgMax(const vector<double>& action_values);

		//Classification of input values (to tile indices)
		string classifyState(const State* state, const int& tile);
		int classifyValue(const double& state_value, const vector<double>& bin_edges);

		// Keep track of how often each state is visited
		map<string,int> state_visits;
		void storeStateVisit(const string& key);
};

#endif //TILECODINGHM_H
