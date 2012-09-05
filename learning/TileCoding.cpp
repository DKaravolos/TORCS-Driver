# include "TileCoding.h"

using namespace std ;

TileCoding::TileCoding( const char * parameterFile, World * w)
{
	Qcount = 0;
    discreteStates = w->getDiscreteStates() ;

    if (!w->getDiscreteActions() || !w->getDiscreteStates()) {

        cout << "Tile coding does not support continuous states or actions." << endl ;
        cout << "Please check which MDP you are using it on." << endl ;
		#ifdef WIN32
			char end_program;
			cin >> end_program;
		#endif
		exit(-4) ;

    } else {
        numberOfActions = w->getNumberOfActions() ;
    }

	string l_filename;
	cout << "Where can I find the tile coding parameters?\n";
	cin >> l_filename;
	//getTileCodingSettings(l_filename); //Gebruik pas als tile coding geïmplementeerd is.

	//Set number of tiles 
	m_numTiles_steer = 2;
	m_numTiles_accel = 2;

	//Set tile size
	m_tileSize_steer = 3;
	m_tileSize_accel = 5;

	//Set values of tiles;
	// De resulterende matrix is  (2* (size - 1)) x (2* (size - 1))


    numberOfStates = w->getNumberOfStates() ;
    Q = new double*[ numberOfStates ] ;

    for ( int s = 0 ; s < numberOfStates ; s++ )
	{
        Q[s] = new double[ numberOfActions ] ;
        for ( int a = 0 ; a < numberOfActions ; a++ )
		{
            Q[s][a] = 0.0 ;
        }
	}

    QTarget = new double[1];
    policy = new double[numberOfActions] ;

}

TileCoding::~TileCoding()
{

}

void TileCoding::getTileCodingSettings(string l_parameterfile)
{
	ifstream is;
	is.open(l_parameterfile);
	if(is.is_open())
	{

		is.close();
	} else {
		cerr <<  "Can't open tile coding parameter file. Quitting\n";
		#ifdef WIN32:
			char end_program;
			cin >> end_program;
		#endif
		exit(-8) ;
	}
}

void TileCoding::update( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma)
{
	if (!state->discrete )
	{
		cerr << "Tile coding does not support continuous states!! Can't update!\n";
		return;
	}

	int at = action->discreteAction ;
	int st = state->discreteState ;
	int st_ = nextState->discreteState ;

	if ( endOfEpisode ) {
		Q[ st ][ at ] += learningRate[0]*( rt - Q[ st ][ at ] ) ; //update with TD error
	} else {
		double maxQs = myMax( Q[ st_ ], numberOfActions ) ;
		Q[ st ][ at ] += learningRate[0]*( rt + gamma*maxQs - Q[ st ][ at ] ) ; //update with TD error
	}
}

double TileCoding::updateAndReturnTDError( State * state, Action * action, double rt, State * nextState, bool endOfEpisode, double * learningRate, double gamma)
{
	if (!state->discrete )
	{
		cerr << "Tile coding does not support continuous states!! Can't update!\n";
		return -1;
	}

	double td_error = 0;
	int at = action->discreteAction ;
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
	return td_error;
}

void TileCoding::readQNN(string nn_file)
{
	for(int action = 0; action < numberOfActions; action++)
	{
		stringstream current_nn;
		current_nn << nn_file << "_action_" << action;
		cNeuralNetwork* nn = new cNeuralNetwork(current_nn.str());
		QNN.push_back(nn);
	}
}