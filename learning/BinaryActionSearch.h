#ifndef BINARY_ACTION_SEARCH_H
#define BINARY_ACTION_SEARCH_H

# include <string>
# include <sstream>
# include <math.h>
# include <vector>
# include "../rlcpp/cNeuralNetwork.h"
# include "../rlcpp/Algorithm.h"
# include "../rlcpp/StateActionUtils.h"
# include "../rlcpp/State.h"
# include "../rlcpp/Action.h"

#include "../utilities/Writer.h"

class BinaryActionSearch: public Algorithm
{
	public:
		//Initialisation, etc.
		BinaryActionSearch(const char * parameterFile, World * w );
		BinaryActionSearch(const char * parameterFile, World * w, string QNN_file);
		~BinaryActionSearch(void);

		void init(Action* action);


		//Algorithm functions:
		void getMaxAction( State * state, Action * action );
		void getRandomAction( State * state, Action * action );
		void explore( State * state, Action * action, double explorationRate, std::string explorationType, bool endOfEpisode );
		void update( State * st, Action * action, double rt, State * st_, bool endOfEpisode, double * learningRate, double gamma  );
		//new:
		double updateAndReturnTDError( State * st, Action * action, double rt, State * st_,
										bool endOfEpisode, double * learningRate, double gamma  ) ;

		void readQNN(std::string QNN_file);
		void writeQNN(std::string QNN_file);

		inline unsigned int getNumberOfLearningRates()	{return 1;}
		inline bool getContinuousStates()				{return true;}
		inline bool getDiscreteStates()					{return true;}
		inline bool getContinuousActions()				{return true;}
		inline bool getDiscreteActions()				{return true;}
		inline const char * getName()	{return "BinaryActionSearch";}

    private:
		//functions:
		void readParameterFile( const char * parameterFile );
        void gaussian( State * state, Action * action, double sigma ) ;
		double gaussianRandom() ;

		//General variables
        double g1, g2 ;
        bool storedGauss ;
		double epsilon, sigma ;
		//double * QTarget ;
		//double * Qs ;
		
		//BAS variables
		int m_nr_actions;
		double* mp_min_vals;
		double* mp_max_vals;
		double* mp_resolution;

		//neural network:
		int nHiddenQ;
		cNeuralNetwork* QNN ;

		//Other datamembers:
		Writer* mp_BAS_log;
};

#endif