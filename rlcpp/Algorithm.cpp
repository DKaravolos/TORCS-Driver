# include "Algorithm.h"

using namespace std ;

Algorithm::Algorithm() {
    discreteStates = false ;
    continuousStates = false ;
    discreteActions = false ;
    continuousActions = false ;
}

Algorithm::~Algorithm() {}

double Algorithm::myMax( double * myarray, int n ) {

    maxX    = myarray[ 0 ] ;

    for ( int i = 1 ; i < n; i++ ) {

        if ( myarray[i] > maxX ) {
            maxX = myarray[i] ;
        }

    }

    return maxX ;

}


int Algorithm::myArgmax( double * myarray, int n ) {

    maxX    = myarray[ 0 ] ;
    maxI    = 0 ;

    for ( int i = 1 ; i < n; i++ ) {

        X = myarray[ i ] ;

        if ( X > maxX ) {
            maxX = X ;
            maxI = i ;
        }

    }

    return maxI ;

}

std::vector<int> Algorithm::argmaxAll( double * myarray, int n ) {

    maxX    = myarray[ 0 ] ;
    std::vector<int> maxI ;
    maxI.push_back( 0 ) ;

    for ( int i = 1 ; i < n; i++ ) {

        X = myarray[ i ] ;

        if ( X > maxX ) {

            maxX = X ;
            maxI.clear() ;
            maxI.push_back( i ) ;

        } else if ( X == maxX ) {

            maxI.push_back( i ) ;

        }

    }

    return maxI ;

}

void Algorithm::egreedy( State * state, Action * action, double epsilon ) {


	#ifdef WIN32
		double random_nr = double(rand())/RAND_MAX ;
	#else
		double random_nr = drand48();
	#endif

    if ( random_nr < epsilon ) {

        getMaxAction( state, action ) ;

    } else {

        getRandomAction( state, action ) ;

    }

}

void Algorithm::read_moveTo( ifstream * ifile, string label ) {
    string temp ;

    while ( temp.compare( label ) != 0 && ifile ) {

        *ifile >> temp ;

        if ( ifile->eof() ) {
            cout << "\nRead error: Could not find label '" << label << "' while reading parameter file." << endl ;
            exit(0) ;
        }

    }
}