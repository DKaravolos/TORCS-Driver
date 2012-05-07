# include "Cacla.h"

#ifdef WIN32
	#include <time.h>
#endif

Cacla::Cacla( const char * parameterFile, World * w ) {

    discreteStates      = w->getDiscreteStates() ;

    actionDimension     = w->getActionDimension() ;

    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

    readParameterFile( parameterFile ) ;

    if ( discreteStates ) {
        numberOfStates = w->getNumberOfStates() ;
        A = new double*[ numberOfStates ] ;
        V = new double[ numberOfStates ] ;

        for ( int s = 0 ; s < numberOfStates ; s++ )
		{
			V[ s ] = 0.0 ;
            A[ s ] = new double[ actionDimension ] ;
            for ( int a = 0 ; a < actionDimension ; a++ )
                A[ s ][ a ] = 0.0 ;
        }
    } else {

        stateDimension = w->getStateDimension() ;

        int layerSizesA[] = { stateDimension, nHiddenQ, actionDimension } ;
        int layerSizesV[] = { stateDimension, nHiddenV, 1 } ;

        ANN = new cNeuralNetwork( 1, layerSizesA ) ;
        VNN = new cNeuralNetwork( 1, layerSizesV ) ;

        VTarget = new double[ 1 ] ;

    }

    storedGauss = false ;

}

Cacla::Cacla( const char * parameterFile, World * w, const char* nn_file ) {

    discreteStates      = w->getDiscreteStates() ;
    actionDimension     = w->getActionDimension() ;

    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

    readParameterFile( parameterFile ) ;

    if ( discreteStates )
	{
		cerr << "Error in Cacla constructor!";
		throw std::invalid_argument("Please do not specify a NN file when using discrete states!!");
    } else {
		//not implemented yet
		cerr << "Calling unimplemented constructor! Exiting....";
		exit(-1);

        stateDimension = w->getStateDimension() ;

        int layerSizesA[] = { stateDimension, nHiddenQ, actionDimension } ;
        int layerSizesV[] = { stateDimension, nHiddenV, 1 } ;

        ANN = new cNeuralNetwork( 1, layerSizesA ) ;
        VNN = new cNeuralNetwork( 1, layerSizesV ) ;

        VTarget = new double[ 1 ] ;
		
    }

    storedGauss = false ;

}

Cacla::~Cacla() {

    if ( discreteStates ) {

        for ( int s = 0 ; s < numberOfStates ; s++ ) {
            delete[] A[s] ;
        }

        delete [] A ;
        delete [] V ;

    } else {

        delete ANN ;
        delete VNN ;

        delete [] VTarget ;

    }

}


void Cacla::readParameterFile( const char * parameterFile ) {

    if ( ! discreteStates ) {

        ifstream ifile ;

        ifile.open( parameterFile, ifstream::in ) ;

        read_moveTo( &ifile, "nn" ) ;

        read_moveTo( &ifile, "nHiddenQ" ) ;
        ifile >> nHiddenQ ;
        read_moveTo( &ifile, "nHiddenV" ) ;
        ifile >> nHiddenV ;

        ifile.close() ;

    }

}

void Cacla::getMaxAction( State * state, Action * action ) {

    double * As ;

    if ( state->discrete ) {

        As = A[ state->discreteState ] ;

    } else {

        As = ANN->forwardPropagate( state->continuousState );

    }

   for ( int a = 0 ; a < actionDimension ; a++ ) {

        action->continuousAction[a] = As[a]  ;

    }

}

void Cacla::getRandomAction( State * state, Action * action ) {

    for ( int a = 0 ; a < actionDimension ; a++ ) {

		#ifdef WIN32
			action->continuousAction[a] = 2.0*double(rand())/RAND_MAX - 1.0 ;
		#else
			action->continuousAction[a] = 2.0*drand48() - 1.0 ;
		#endif
    }

}


void Cacla::explore( State * state, Action * action, double explorationRate, string explorationType, bool endOfEpisode ) {

    if ( explorationType.compare("boltzmann") == 0 ) {

        cerr << "Boltzmann exploration is as of yet undefined for Cacla." << endl ;
        exit(-1) ;

    } else if ( explorationType.compare("egreedy") == 0  ) {

        egreedy( state, action, explorationRate ) ;

    } else if ( explorationType.compare("gaussian") == 0  ) {

        gaussian( state, action, explorationRate ) ;

    } else {

        cerr << "Warning, no exploreType: " << explorationType << endl ;
        exit(-1) ;

    }

}

double Cacla::gaussianRandom() {
    // Generates gaussian (or normal) random numbers, with mean = 0 and
    // std dev = 1. Used for gaussian exploration.

    if ( storedGauss ) {

        storedGauss = false ;

        return g2 ;

    } else {

        double x, y ;

        double z = 1.0 ;

        while ( z >= 1.0 ) {
			#ifdef WIN32
				x = 2.0*double(rand())/RAND_MAX - 1.0;
				y = 2.0*double(rand())/RAND_MAX - 1.0;
			#else
				x = 2.0*drand48() - 1.0;
				y = 2.0*drand48() - 1.0;
			#endif
            z = x * x + y * y;
        }

        z = sqrt( -2.0 * log( z ) / z );

        g1 = x * z;
        g2 = y * z;

        storedGauss = true ;

        return g1 ;

    }

}

void Cacla::gaussian( State * state, Action * action, double sigma ) {

    getMaxAction( state, action ) ;

    for ( int a = 0 ; a < actionDimension ; a++ ) {

        action->continuousAction[a] += sigma*gaussianRandom() ;

    }

}

void Cacla::update( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma  ) {

    double * at = action->continuousAction ;

    if ( state->discrete ) {

        int st      = state->discreteState ;
        int st_     = nextState->discreteState ;

        double Vt = V[ st ] ;

        if ( endOfEpisode ) {

            V[ st ]       += learningRate[1]*( rt - V[ st ] ) ;

        } else {

            V[ st ]       += learningRate[1]*( rt + gamma*V[ st_ ] - V[ st ] ) ;

        }

        if ( V[ st ] > Vt ) {

            for ( int a = 0 ; a < actionDimension ; a++ ) {

                A[ st ][ a ] += learningRate[0]*( at[ a ] - A[ st ][ a ] ) ;

            }

        }

    } else {

        double * st = state->continuousState ;
        double * st_ = nextState->continuousState ;

        if ( endOfEpisode ) {

            VTarget[ 0 ] = rt ;

        } else {

            double Vs_   = VNN->forwardPropagate( st_ )[0] ;

            VTarget[ 0 ] = rt + gamma*Vs_ ;

        }

        double Vt = VNN->forwardPropagate( st )[0] ;

        VNN->backPropagate( st, VTarget, learningRate[1] ) ;

        if ( VTarget[0] > Vt ) {

            ANN->backPropagate( st, at, learningRate[0] ) ;

        }

    }

}


unsigned int Cacla::getNumberOfLearningRates() {

    return 2 ;

}

bool Cacla::getContinuousStates() {

    return true ;

}

bool Cacla::getDiscreteStates() {

    return true ;

}

bool Cacla::getContinuousActions() {

    return true ;

}

bool Cacla::getDiscreteActions() {

    return false ;

}

const char * Cacla::getName() {

    return "Cacla" ;

}