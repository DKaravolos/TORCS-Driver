#ifndef STATE
#define STATE

struct State {

    bool continuous ;
    bool discrete ;

    int stateDimension ; //als continuous (bij mij: 13)
    int numberOfStates ; //als discrete

    int discreteState ;
    double * continuousState ; //dit is gevuld met featureVector. stateDimension == lengte van array.

	int time_step;
};

#endif //STATE
