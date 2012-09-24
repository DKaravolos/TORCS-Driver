#ifndef TILECODING_H
#define TILECODING_H
# include <iostream>
# include <sstream>
# include <string>
# include <vector>
# include "../rlcpp/StateActionAlgorithm.h"
# include "../rlcpp/State.h"

using namespace std;
class TileCoding : public StateActionAlgorithm {
    public:
        TileCoding(World * w);
		TileCoding(World * w, const char* qtable_file);
        ~TileCoding();

        void update	(State * st, Action * action, double rt, State * st_,
					bool endOfEpisode, double * learningRate, double gamma);

		double updateAndReturnTDError	(State * st, Action * action, double rt, State * st_,
										bool endOfEpisode, double * learningRate, double gamma);
		
		void getMaxAction(State* state, Action* action);
		void loadQTable (string filename);
		void writeQTable (string filename);

		inline unsigned int getNumberOfLearningRates()	{return 1;}
		inline const char * getName()					{return "TileCoding" ;}
	private:
		//QTable:
		//tile,	speed,	trackpos,angle,		dist1, dist2, dist3, dist4, dist5,		actions
		vector<	vector<	vector<	 vector<	vector<vector<vector<vector<vector<	vector<double>>>>>>>>>> mp_tiles;

		//Container for indices of mp_tiles
		vector<int*> m_state_bins;
		vector<int*> m_next_state_bins;

		//tile info
		int m_numTiles;
		int m_tileNum_speed;
		int m_tileNum_pos;
		int m_tileNum_angle;
		int m_tileNum_dist;

		//edges of squares in tiles
		vector<vector<double>> m_speed_edges;
		vector<vector<double>> m_trackPos_edges;
		vector<vector<double>> m_angle_edges;
		vector<vector<double>> m_dist_edges;

		//functions
		//void getTileCodingSettings(string parameterfile);
		void init(World* world);
		void setEdges();
		void getEdgesFromFile(string filename);
		double maxQOfState(vector<int*> state);

		double myTCMax(vector<double> action_values);
		int myTCArgMax(vector<double> action_values);

		//Classification of input values (to tile indices)
		int* classifyState(State* state, int tile);
		int classifyValue(double state_value, vector<double> bin_edges);
};

#endif //TILECODING_H
