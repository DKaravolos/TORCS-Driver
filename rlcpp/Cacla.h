#ifndef CACLA_H
#define CACLA_H

# include <string>
# include <math.h>
# include <vector>
# include <time.h>
# include "cNeuralNetwork.h"
# include "Algorithm.h"
# include "StateActionUtils.h"

#include <sstream>
#include "../utilities/Writer.h"

class Cacla : public Algorithm {
    public:
        Cacla( const char * parameterFile, World * w ) ;
		Cacla( const char * parameterFile, World * w, const char* annFile, const char* vnnFile ) ;
        ~Cacla() ;

        void readParameterFile( const char * parameterFile ) ;
        void getMaxAction( State * state, Action * action ) ;
        void getRandomAction( State * state, Action * action ) ;
        void explore( State * state, Action * action, double explorationRate, string explorationType, bool endOfEpisode ) ;
        void update( State * st, Action * action, double rt, State * st_, bool endOfEpisode, double * learningRate, double gamma  ) ;

		void readNN(std::string ANN_file, std::string VNN_file);
		void writeNN(std::string ANN_file, std::string VNN_file);

        unsigned int getNumberOfLearningRates() ;
        bool getContinuousStates() ;
        bool getDiscreteStates() ;
        bool getContinuousActions() ;
        bool getDiscreteActions() ;
        const char * getName() ;

    private:
        void gaussian( State * state, Action * action, double sigma ) ;

        double gaussianRandom() ;
        double g1, g2 ;
        bool storedGauss ;

        int nHiddenQ, nHiddenV ;
        double epsilon, sigma ;
        double * V ;
        double ** A ;
        cNeuralNetwork * ANN ;
        cNeuralNetwork * VNN ;
        double * VTarget ;

		Writer* mp_critic_log;
};

#endif //CACLA_H
