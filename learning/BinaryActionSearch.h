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
	enum GetMaxOption {ORIGINAL, INVERTED_LOOPS};
	public:
		//Initialisation, etc.
		BinaryActionSearch	(const char * parameterFile, World * w);
		BinaryActionSearch	(const char * parameterFile, World * w, string QNN_file);
		~BinaryActionSearch	(void);
		void init(Action* action);

		//Algorithm functions:
		void getRandomAction		(State * state, Action * action);
		virtual void getMaxAction	(State * state, Action * action);
		void getMaxAction			(State * state, Action * action, GetMaxOption option);

		void explore (State * state, Action * action, double explorationRate,
					  std::string explorationType, bool endOfEpisode);

		virtual void update (State * st, Action * action, double rt, State * st_,
							 bool endOfEpisode, double * learningRate, double gamma);

		virtual double updateAndReturnTDError  (State * st, Action * action, double rt, State * st_,
												bool endOfEpisode, double * learningRate, double gamma);

		//IO functions
		virtual void writeQNN(std::string QNN_file);

		//Getters
		inline unsigned int getNumberOfLearningRates()	{return 1;}
		inline bool getContinuousStates()				{return true;}
		inline bool getDiscreteStates()					{return true;}
		inline bool getContinuousActions()				{return true;}
		inline bool getDiscreteActions()				{return true;}
		inline const char * getName()	{return "BinaryActionSearch";}

    protected:
		//Init functions:
		void _initVars(const char* parameterFile, World*w);
		void readParameterFile( const char * parameterFile );

		//IO functions
		virtual void readQNN(std::string QNN_file);

		//Explore functions
        void gaussian( State * state, Action * action, double sigma ) ;
		double gaussianRandom() ;

		//GetMax functions
		void originalGetMax			(State * state, Action * action);
		//void withRootsGetMax		(State * state, Action * action);
		void invertedLoopsGetMax	(State * state, Action * action);

		//Update functions
		virtual void updateAugmentedStates(double* NN_input, int action, double learningRate);

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
		vector<vector<double>*> m_deltas;
		vector<vector<double>*> m_last_sequence;
		vector<double>* m_NN_input;

		//neural network:
		int nHiddenQ;
		//vector<cNeuralNetwork*> QNN;
		vector<cNeuralNetwork*> QNN_left ;
		vector<cNeuralNetwork*> QNN_right ;

		//Other datamembers:
		Writer* mp_BAS_log;
};

#endif  //BINARY_ACTION_SEARCH_H