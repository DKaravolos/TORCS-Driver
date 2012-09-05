#ifndef TILECODING_H
#define TILECODING_H
# include <iostream>
# include <sstream>
# include <string>
# include "../rlcpp/StateActionAlgorithm.h"

class TileCoding : public StateActionAlgorithm {
    public:
        TileCoding( const char * parameterFile, World * w ) ;
        ~TileCoding() ;

        void update	(State * st, Action * action, double rt, State * st_,
					bool endOfEpisode, double * learningRate, double gamma);

		double updateAndReturnTDError	(State * st, Action * action, double rt, State * st_,
										bool endOfEpisode, double * learningRate, double gamma);

		inline unsigned int getNumberOfLearningRates()	{return 1;}
		inline const char * getName()					{return "TileCoding" ;}
		void readQNN(string nn_file);

	private:
		int Qcount;
		//tile info steer
		int m_numTiles_steer;
		int m_tileSize_steer;

		//tile info accel
		int m_numTiles_accel;
		int m_tileSize_accel;



		void getTileCodingSettings(string parameterfile);

};

#endif //QLEARNING_H
