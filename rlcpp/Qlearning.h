#ifndef QLEARNING_H
#define QLEARNING_H
# include <iostream>
# include <sstream>
# include <string>
# include "StateActionAlgorithm.h"

class Qlearning : public StateActionAlgorithm {
    public:
        Qlearning( const char * parameterFile, World * w ) ;
		Qlearning( const char * parameterFile, World * w, const char * nnFile) ;
        ~Qlearning() ;
        void readParameterFile( const char * parameterFile ) ;
        void update(State * st, Action * action, double rt, State * st_,
					bool endOfEpisode, double * learningRate, double gamma);

		double updateAndReturnTDError(State * st, Action * action, double rt, State * st_,
										bool endOfEpisode, double * learningRate, double gamma);
        unsigned int getNumberOfLearningRates() ;
        const char * getName() ;
		int readQNN(string nn_file);

	private:
		int Qcount;

};

#endif //QLEARNING_H
