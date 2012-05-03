#ifndef QALGORITHM_H
#define QALGORITHM_H

# include <string>
#include <sstream>
# include <math.h>
# include <vector>
# include "cNeuralNetwork.h"
# include "Algorithm.h"
# include "StateActionUtils.h"

class StateActionAlgorithm : public Algorithm {
    public:
        StateActionAlgorithm() ;
        virtual ~StateActionAlgorithm() {}
        void getMaxAction( State * state, Action * action ) ;
        void explore( State * state, Action * action, double explorationRate, string explorationType, bool endOfEpisode ) ;
        bool getContinuousStates() ;
        bool getDiscreteStates() ;
        bool getContinuousActions() ;
        bool getDiscreteActions() ;

		void writeQNN(std::string QNNFileName);

    protected:
        void getMaxActionFirst( State *, Action * action ) ;
        void getMaxActionRandom( State *, Action * action ) ;
        void getRandomAction( State * state, Action * action ) ;

        void setQs( State * state, Action * action ) ;

        void egreedy( State * state, Action * action, double epsilon ) ;
        void boltzmann( State  * state, Action * action, double tau ) ;
        void boltzmann( Action * action, double tau ) ;
        void gaussian( State * state, Action * action, double tau ) ;

        int nHiddenQ ;
        double ** Q ;
        vector< cNeuralNetwork * > QNN ;
        double * Qs ;
        double * QTarget ;
        double * policy ;
};

#endif //QALGORITHM_H
