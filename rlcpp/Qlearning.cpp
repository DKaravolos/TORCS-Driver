# include "Qlearning.h"
#ifdef WIN32
	#include <time.h>
#endif

using namespace std ;

Qlearning::Qlearning( const char * parameterFile, World * w ) {
	Qcount = 0;
    discreteStates      = w->getDiscreteStates() ;
	//cout << "In Q-Learning constructor: mp_world is at " << w << endl;
    if ( !w->getDiscreteActions() ) {

        cout << "Q-learning does not support continuous actions." << endl ;
        cout << "Please check which MDP you are using it on." << endl ;
		#ifdef WIN32:
			char end_program;
			cin >> end_program;
		#endif
		exit(0) ;

    } else {

        numberOfActions = w->getNumberOfActions() ;

    }

    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

    readParameterFile( parameterFile ) ;

    if ( discreteStates ) {

        numberOfStates = w->getNumberOfStates() ;

        Q = new double*[ numberOfStates ] ;

        for ( int s = 0 ; s < numberOfStates ; s++ ) {

            Q[s] = new double[ numberOfActions ] ;

            for ( int a = 0 ; a < numberOfActions ; a++ ) {
                Q[s][a] = 0.0 ;
            }

        }

    } else {

        stateDimension = w->getStateDimension() ;

        int layerSizesA[] = { stateDimension, nHiddenQ, 1 } ;
		cout << "QL: stateDimension :" << stateDimension << endl;
		cout << "QL: hidden layer size :" << nHiddenQ << endl;

        for ( int a = 0 ; a < numberOfActions ; a++ ) {

            QNN.push_back( new cNeuralNetwork( 1, layerSizesA ) ) ;

        }

        Qs = new double[ numberOfActions ] ;
		cout << "QL: numberOfActions :" << numberOfActions << endl;
    }

    QTarget = new double[ 1 ] ;
    policy = new double[ numberOfActions ] ;

}


Qlearning::Qlearning( const char * parameterFile, World * w, const char * nn_file ) {
	Qcount = 0;
    discreteStates = w->getDiscreteStates() ;
    if ( !w->getDiscreteActions() ) {

        cout << "Q-learning does not support continuous actions." << endl ;
        cout << "Please check which MDP you are using it on." << endl ;
		#ifdef WIN32:
			char end_program;
			cin >> end_program;
		#endif
		exit(0) ;

    } else {
        numberOfActions = w->getNumberOfActions() ;
    }

    #ifdef WIN32
		srand( clock() ) ;
	#else
		srand48( clock() ) ;
	#endif

    readParameterFile( parameterFile ) ;

    if ( discreteStates ) {

        numberOfStates = w->getNumberOfStates() ;

        Q = new double*[ numberOfStates ] ;

        for ( int s = 0 ; s < numberOfStates ; s++ ) {

            Q[s] = new double[ numberOfActions ] ;

            for ( int a = 0 ; a < numberOfActions ; a++ ) {
                Q[s][a] = 0.0 ;
            }

        }

    } else {

        stateDimension = w->getStateDimension() ;
		//int QNN_vector_size = readQNN(nn_file);

  //      if(QNN_vector_size != numberOfActions) {
		//	cerr << "Oh no! The input QNN does not have size numberOfActions!";
		//	#ifdef WIN32:
		//		char end_program;
		//		cin >> end_program;
		//	#endif
		//	exit(-1) ;
		//}
        Qs = new double[ numberOfActions ] ;
		cout << "QL: numberOfActions :" << numberOfActions << endl;
    }

    QTarget = new double[ 1 ] ;
    policy = new double[ numberOfActions ] ;

}

Qlearning::~Qlearning() {

    if ( discreteStates ) {

        for ( int s = 0 ; s < numberOfStates ; s++ ) {

            delete [] Q[s] ;

        }

        delete [] Q ;

    } else {

        for ( int a = 0 ; a < numberOfActions ; a++ ) {

            delete QNN[a] ;

        }

        QNN.clear() ;

        delete [] Qs ;

    }

    delete [] QTarget ;
    delete [] policy ;

}

void Qlearning::readParameterFile( const char * parameterFile ) {

    if ( !discreteStates ) {

        ifstream ifile ;

        ifile.open( parameterFile, ifstream::in ) ;
		if(ifile.is_open())
		{
			read_moveTo( &ifile, "nn" ) ;

			read_moveTo( &ifile, "nHiddenQ" ) ;
			ifile >> nHiddenQ ;

			ifile.close() ;
		}
		else
		{
			cout << "\nCould not open parameter file. Please check the filename.\n";
			#ifdef WIN32
				char end;
				cin >> end;
			#endif
			exit(-1);
		}
    }

}

void Qlearning::update( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma  ) {
	
    int at = action->discreteAction ;

    if ( state->discrete ) {

        int st = state->discreteState ;
        int st_ = nextState->discreteState ;

        if ( endOfEpisode ) {

            Q[ st ][ at ] += learningRate[0]*( rt - Q[ st ][ at ] ) ; //update with TD error

        } else {

            double maxQs = myMax( Q[ st_ ], numberOfActions ) ;

            Q[ st ][ at ] += learningRate[0]*( rt + gamma*maxQs - Q[ st ][ at ] ) ; //update with TD error

        }


    } else {

        double * st = state->continuousState ;
        double * st_ = nextState->continuousState ;

        if ( endOfEpisode ) {

            QTarget[ 0 ] = rt ;

        } else {

            for ( int a = 0 ; a < numberOfActions ; a++ ) {
                Qs[a] = QNN[a]->forwardPropagate( st_ )[0] ;
				/*if (Qcount % 100 == 0)
					cout << Qcount << " Qvalue of action :"<< a << " : " << Qs[a] << endl;*/
            }

            double maxQs = myMax( Qs, numberOfActions ) ;
            QTarget[ 0 ] = rt + gamma*maxQs ;

        }

        QNN[ at ]->backPropagate( st, QTarget, learningRate[0] ) ;
		
		//Qcount++;
		//if(Qcount % 1000 == 0)
		//	Qcount = 0;
    }

}

double Qlearning::updateAndReturnTDError( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma  ) {
	
    int at = action->discreteAction ;
	double td_error = 0;
    if ( state->discrete ) {

        int st = state->discreteState ;
        int st_ = nextState->discreteState ;

        if ( endOfEpisode ) {
			td_error = rt - Q[ st ][ at ];
            Q[ st ][ at ] +=  learningRate[0] * td_error; //update with TD error

        } else {

            double maxQs = myMax( Q[ st_ ], numberOfActions);
			td_error = rt + gamma*maxQs - Q[ st ][ at ];
            Q[ st ][ at ] +=  learningRate[0] * td_error ; //update with TD error

        }


    } else {

        double * st = state->continuousState ;
        double * st_ = nextState->continuousState ;

        if ( endOfEpisode ) {

            QTarget[ 0 ] = rt ;

        } else {

            for ( int a = 0 ; a < numberOfActions ; a++ ) {
                Qs[a] = QNN[a]->forwardPropagate( st_ )[0] ;
				/*if (Qcount % 100 == 0)
					cout << Qcount << " Qvalue of action :"<< a << " : " << Qs[a] << endl;*/
            }

            double maxQs = myMax( Qs, numberOfActions ) ;

            QTarget[ 0 ] = rt + gamma*maxQs ;
			double current_val = QNN[at]->forwardPropagate( st )[0];
			td_error = QTarget[0] -current_val;
        }

        QNN[ at ]->backPropagate( st, QTarget, learningRate[0] ) ;
		
		//Qcount++;
		//if(Qcount % 1000 == 0)
		//	Qcount = 0;
    }
	return td_error;
}

unsigned int Qlearning::getNumberOfLearningRates() {

    return 1 ;

}

const char * Qlearning::getName() {

    return "Qlearning" ;

}

int Qlearning::readQNN(string nn_file)
{
	ifstream ifile;

	for(int action = 0; action < numberOfActions; action++)
	{
		stringstream current_nn;
		current_nn << nn_file << "_action_" << action;
		ifile.open(current_nn.str());
		string first_line;
		if(ifile.is_open())
		{
			//Check if we are reading a properly formed QNN file
			ifile >> first_line;
			if(first_line.compare("QNN") == 0)
				cerr << "I am reading a QNN from a file. fancy!\n";
			else {
				cerr << "I should be looking at a QNN file, but I am not.\n";
				throw std::invalid_argument("This file does not contain a QNN!");
				exit(-1);
			}
			cNeuralNetwork* nn = new cNeuralNetwork(ifile);
			QNN.push_back(nn);

			ifile.close();
		} else {
			cerr << "Can't open file "<< nn_file << "!\n";
			throw std::invalid_argument("Can't open specified file");
		}
	}
}