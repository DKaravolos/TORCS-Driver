#ifndef EXPERIMENT_H
#define EXPERIMENT_H

# include <iostream>
# include <string>
# include <sstream>
# include <vector>

# include "State.h"
# include "Action.h"
# include "World.h"
# include "Algorithm.h"

# include "Qlearning.h"
# include "QVlearning.h"
# include "Sarsa.h"
# include "Acla.h"
# include "Cacla.h"

class Experiment {
    public:
		enum Configuration
		{
			QLEARNING,
			CACLA,
			BAS,
			QOS,
			TILECODING
		};

		
		friend class BASLearningInterface;
		friend class CaclaLearningI;
		friend class LearningInterface;
		friend class RLInterface;
		friend class TCLearningInterface;
		friend class QOSLearningInterface;
		
		Experiment(){}
		Experiment(Configuration config);
        virtual ~Experiment() ;
        bool initializeState( State * state, Algorithm * algorithm, World * world ) ;
        bool initializeAction( Action * action, Algorithm * algorithm, World * world ) ;
        void getParametersAndRunExperiments( World * world ) ;

		inline void setEpsilon(double eps_in)		{epsilon = eps_in;}
		inline void setTau(double tau_in)			{tau = tau_in;}
		inline void setSigma (double sigma_in)		{sigma = sigma_in;}
		inline void setGamma(double gamma_input)	{gamma = gamma_input;}
		inline void setLearningRate(double lr)		{learningRate[0] = lr;}
		inline void setLearningRates(double lr0, double lr1) {learningRate[0] = lr0; learningRate[1] = lr1;}
		inline void setBoltzmann(bool input)	{boltzmann = input;}
		inline void setEGreedy(bool input)		{egreedy = input;}
		inline void setGaussian(bool input)		{sigma = input;}

		inline double getEpsilon()		{return epsilons[0];}
		inline double getTau()			{return taus[0];}
		inline double getSigma()		{return sigmas[0];}
		inline double getGamma()		{return gamma;}
		inline double getLearningRate() {return learningRate[0];}
		inline bool getBoltzmann()		{return boltzmann;}
		inline bool getEGreedy()		{return egreedy;}
        
    protected:
        double * runExperiment( World * world ) ;
        void explore( State * state, Action * action ) ;
        void storeRewards( const char *, double * , int ) ;
        void setAlgorithm( string algorithmName, World * world ) ;
        void readParameterFile( ) ;
		void readParameterFile(std::string paramFile ) ;
        void read_moveTo( ifstream *, string ) ;
        vector< double > read_doubleArray( string ) ;
        
        Algorithm * algorithm ;
        World * world ;
        
        State * state ;
        State * nextState ;
        Action * action ;
        Action * nextAction ;
        
        int nExperiments, nAlgorithms;
        int nSteps, nEpisodes, nMaxStepsPerEpisode, nResults ;
        int nTrainSteps, nTrainEpisodes, nMaxStepsPerTrainEpisode, nTrainResults, trainStorePer ;
        int nTestSteps, nTestEpisodes, nMaxStepsPerTestEpisode, nTestResults, testStorePer ;
        int stateDimension, actionDimension ;
        bool discreteStates, discreteActions, endOfEpisode ;
        bool storePerStep, storePerEpisode ;
        bool boltzmann, egreedy, gaussian ;
        int nLearningRates ;
        
        const char * parameterFile ;
        string parameterPath ;
        string algorithmName ;
        vector< string > algorithms ;
        vector< double > taus ;
        vector< double > epsilons ;
        vector< double > sigmas ;
        string learningRateDecreaseType ;
        double * learningRate ;
        vector< vector < double > >  learningRates ;
        vector< double >  gammas ;
        double tau, epsilon, sigma, gamma ;
        
        bool train ;
        
    
};

#endif //EXPERIMENT_H

