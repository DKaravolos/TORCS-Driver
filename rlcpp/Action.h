#ifndef ACTION
#define ACTION

struct Action {
    bool continuous ;
    bool discrete ;

    int actionDimension ;
    int numberOfActions ;

    int discreteAction ;
    double * continuousAction ;

	double* min_val; //don't forget to init when using BAS!
	double* max_val; //don't forget to init when using BAS!
};

#endif //Action
