using namespace std;

void read_moveTo( ifstream * ifile, string label ) {
    string temp ;

    while ( temp.compare( label ) != 0 && ifile ) {

        *ifile >> temp ;

        if ( ifile->eof() ) {
            cout << "Read error: Could not find label '" << label << "' while reading parameter file '" << parameterFile << "'" << endl ;
            exit(0) ;
        }

    }
}

vector< double > read_doubleArray( string temp ) {

    vector< double > parameters ;

    istringstream iss( temp ) ;

    double parameter ;

    while( iss >> parameter ) {

        parameters.push_back( parameter ) ;

    }

    return parameters ;

}

void readParameterFile( ) {

    ifstream ifile ;

    ifile.open( parameterFile, ifstream::in ) ;

    string temp ;

    read_moveTo( &ifile, "nExperiments" ) ;
    ifile >> nExperiments ;

    read_moveTo( &ifile, "steps" ) ;

    read_moveTo( &ifile, "nTrainSteps" ) ;
    ifile >> nTrainSteps ;

    read_moveTo( &ifile, "trainStorePer" ) ;
    ifile >> trainStorePer ;

    read_moveTo( &ifile, "nTestSteps" ) ;
    ifile >> nTestSteps ;

    read_moveTo( &ifile, "testStorePer" ) ;
    ifile >> testStorePer ;

    read_moveTo( &ifile, "nTrainEpisodes" ) ;
    ifile >> nTrainEpisodes ;

    read_moveTo( &ifile, "nTestEpisodes" ) ;
    ifile >> nTestEpisodes ;

    read_moveTo( &ifile, "nMaxStepsPerTrainEpisode" ) ;
    ifile >> nMaxStepsPerTrainEpisode ;

    read_moveTo( &ifile, "nMaxStepsPerTestEpisode" ) ;
    ifile >> nMaxStepsPerTestEpisode ;


    if ( nTrainSteps == 0 ) {

        nTrainSteps     = nTrainEpisodes*nMaxStepsPerTrainEpisode ;
        nTrainResults   = nTrainSteps/trainStorePer ;
        storePerStep    = false ;
        storePerEpisode = true ;

    } else if ( nMaxStepsPerTrainEpisode == 0 ) {

        nTrainEpisodes  = nTrainSteps ;
        nTrainResults   = nTrainSteps/trainStorePer ;
        storePerStep    = true ;
        storePerEpisode = false ;

    }

    if ( nTestSteps == 0 ) {

        nTestSteps     = nTestEpisodes*nMaxStepsPerTestEpisode ;
        nTestResults   = nTestSteps/testStorePer ;
        storePerStep    = false ;
        storePerEpisode = true ;

    } else if ( nMaxStepsPerTestEpisode == 0 ) {

        nTestEpisodes  = nTestSteps ;
        nTestResults   = nTestSteps/testStorePer ;
        storePerStep    = true ;
        storePerEpisode = false ;

    }


    read_moveTo( &ifile, "algorithm" ) ;

    read_moveTo( &ifile, "algorithms" ) ;
    getline( ifile, temp ) ;
    istringstream iss( temp ) ;

    while ( iss >> temp ) {
        algorithms.push_back( temp ) ;
    }

    read_moveTo( &ifile, "tau" ) ;
    getline( ifile, temp ) ;
    taus = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "epsilon" ) ;
    getline( ifile, temp ) ;
    epsilons = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "sigma" ) ;
    getline( ifile, temp ) ;
    sigmas = read_doubleArray( temp ) ;

    read_moveTo( &ifile, "learningRates" ) ;

    read_moveTo( &ifile, "decreaseType" ) ;
    ifile >> learningRateDecreaseType ;

    read_moveTo( &ifile, "nLearningRates" ) ;
    ifile >> nLearningRates ;

    for ( int l = 0 ; l < nLearningRates ; l++ ) {

        read_moveTo( &ifile, "learningRate" ) ;
        getline( ifile, temp ) ;
        learningRates.push_back( read_doubleArray( temp ) ) ;

    }

    read_moveTo( &ifile, "discount" ) ;

    read_moveTo( &ifile, "gamma" ) ;
    getline( ifile, temp ) ;
    gammas = read_doubleArray( temp ) ;

}