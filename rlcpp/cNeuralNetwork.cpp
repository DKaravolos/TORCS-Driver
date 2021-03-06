# include "cNeuralNetwork.h"
# include <cstdlib>
#include <stdexcept>
//# include <math.h>

#define THRESHOLD 2
#define TANH 1
#define LINEAR 0
#define SIZEOFDOUBLE sizeof( double ) 


#ifdef WIN32
	#include <time.h>
#endif

using namespace std ;

cNeuralNetwork::cNeuralNetwork( const char * nnFile ) {
	pointerInit();
    readNetwork( nnFile ) ;
}

cNeuralNetwork::cNeuralNetwork( string nnFile ) {
	pointerInit();
    readNetwork( nnFile ) ;
}

cNeuralNetwork::cNeuralNetwork(ifstream& is) {
	pointerInit();
    readNetwork(is) ;
}

void cNeuralNetwork::pointerInit()
{
	    layerSize = NULL;
        layerFunctionInts  = NULL;
        formerOut  = NULL;
        formerIn  = NULL;
        nextIn  = NULL;
        nextOut  = NULL;
        iError  = NULL;
        oError  = NULL;
        w  = NULL;
        inputIn  = NULL;
        inputOut  = NULL;
        outputIn  = NULL;
        outputOut  = NULL;
        error  = NULL;
        layerFunction  = NULL;
        weights  = NULL;
        layerIn  = NULL;
        layerOut  = NULL;

		//_createLog("log_files/NN_fromPointer_log");
}

cNeuralNetwork::cNeuralNetwork( int nLayersInit, int * layerSizeInit ) {
    int * layerFunctions = new int[ nLayersInit + 2 ] ;
    for ( int l = 1 ; l < nLayersInit + 1 ; l++ ) {
        layerFunctions[ l ] = TANH ;
    }
    layerFunctions[ 0 ] = LINEAR ;
    layerFunctions[ nLayersInit + 1 ] = LINEAR ;
    
    init( nLayersInit, layerSizeInit, layerFunctions ) ;
    
    delete [] layerFunctions ;
    
}
    
cNeuralNetwork::cNeuralNetwork( int nLayersInit, int * layerSizeInit, int * layerFunctionInit ) {
    // int nLayersInit : the number of hidden layers (So for input, hidden, output -> 1)
    // int * layerSizeInit : the number of nodes per layer (# nodes layers = # weights layers + 1)
    //                       (size should be nLayersInit + 2)
    // int * layerFunctionInit : code, specifying the function per (node) layer

    //~ cout << "constructor NN L\n" ;
    init( nLayersInit, layerSizeInit, layerFunctionInit ) ;
}

void cNeuralNetwork::init( int nLayersInit, int * layerSizeInit, int * layerFunctionInit ) {
    #ifdef WIN32
		srand( unsigned int(time(0)) ) ;
	#else
		srand48( time(0) ) ;
	#endif

	//Create adaptive NN log file, in case of multiple networks
	//_createLog("log_files/NN_log");


    nLayers = nLayersInit + 2 ; // # node layers == # hidden layers + 2
    nInput  = layerSizeInit[ 0 ] ;
    nOutput = layerSizeInit[ nLayers - 1 ] ;
    
    //~ cout << "nInput" << nInput << endl ;
    //~ cout << "nOutput" << nOutput << endl ;
    //~ cout << "nLayers" << nLayers << endl ;
    
    layerFunction       = new cFunction*[ nLayers ] ;
    layerFunctionInts   = new int[ nLayers ] ;
    weights             = new Matrix*[ nLayers - 1 ] ; // # weights layers = # node layers - 1
    layerIn             = new Matrix*[ nLayers ] ; 
    layerOut            = new Matrix*[ nLayers ] ;
    layerSize           = new int[ nLayers ] ;
    
    for(int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        weights[ l ]  = new Matrix( layerSizeInit[ l ] + 1, layerSizeInit[ l + 1 ] ) ; // layerSize[ l ] + 1, for bias
    }
    for(int l = 0 ; l < nLayers ; l++ ) {
        layerSize[ l ] = layerSizeInit[ l ] ;
        layerIn[ l ]  = new Matrix( layerSize[ l ] ) ;
        layerOut[ l ] = new Matrix( layerSize[ l ] ) ;
        
        layerFunctionInts[ l ] = layerFunctionInit[ l ] ;
        if ( layerFunctionInit[ l ] == TANH ) {
            layerFunction[ l ] = new cTanH() ;
        } else if ( layerFunctionInit[ l ] == LINEAR ) {
            layerFunction[ l ] = new cLinear() ;
        } else {
            cout << "WARNING: Unknown layer function type: " ;
            cout << layerFunctionInit[ l ] << '\n' ;
            cout << "layer: " << l << '\n' ;
			#ifdef WIN32
					char end;
					cin>>end;
			#endif
            exit(-2) ;
        }
    }
    
    // Initialise the weights randomly between -0.3 and 0.3
    randomizeWeights( -0.3, 0.3) ;
    recentlyUpdated = true ;
}

cNeuralNetwork::~cNeuralNetwork( ) {
    //~ cout << "destructor NN\n" ;
	for(int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
       delete weights[ l ];
    }
    for(int l = 0 ; l < nLayers ; l++ ) {
       delete layerIn[ l ];
       delete layerOut[ l ];
       delete layerFunction[ l ];
	}
    delete [] layerIn ;
    delete [] layerOut ;
    delete [] layerFunction ; 
    delete [] layerFunctionInts ;
    delete [] layerSize ;
    delete [] weights ;

	//delete mp_NN_log;
}

void        cNeuralNetwork::changeFunction( int layer, int function ) {
    delete layerFunction[ layer ] ;
    
    if ( function == TANH ) {
        
        layerFunction[ layer ] = new cTanH() ;
    
    } else if ( function == LINEAR ) {
        
        layerFunction[ layer ] = new cLinear() ;
    
    } else if ( function == THRESHOLD ) {
        
        layerFunction[ layer ] = new cThreshold() ;
    
    }
    
    recentlyUpdated = true ;
}
   
void cNeuralNetwork::_createLog(string f_name)
{
	//Create adaptive NN log file, in case of multiple networks
	ifstream check;
	
	for(int f_nr = 0; f_nr <=20; f_nr++)
	{
		stringstream file_name;
		file_name << f_name << f_nr << ".txt";
		check.open(file_name.str().c_str());
		if(!check.is_open())
		{
			mp_NN_log = new Writer(file_name.str());
			break;
		}
		else
			check.close();
	}
}

void        cNeuralNetwork::_forwardPropLayer( int layer ) {

    w = weights[ layer ]->getPtr() ; // The weights of the current layer
    
    nFormer = layerSize[ layer ] ;       // # nodes in the former nodes layer
    nNext   = layerSize[ layer + 1 ] ;   // # nodes in the next nodes layer
    
    formerOut  = layerOut[ layer ]->getPtr() ;      // The output of the former layer
    nextIn     = layerIn[ layer + 1 ]->getPtr() ;   // The input of the next layer (to be set)
    nextOut    = layerOut[ layer + 1 ]->getPtr() ;  // The output of the next layer (follows from the input and function)
    
    cFunction * f = layerFunction[layer+1] ;                 // The function on the next layer

    // Initialise inputs to the next layer to zero
    for(int o = 0 ; o < nNext ; o++ ) {
        nextIn[ o ] = 0.0 ;
    }
    
    // initialise counter for the weights
    int wPos = 0 ;
    // add weighted inputs
    for(int i = 0 ; i < nFormer ; i++ ) {
        if ( formerOut[ i ] != 0.0) {
            for(int o = 0 ; o < nNext ; o++ ) {
                nextIn[ o ] += w[ wPos ] * formerOut[ i ] ;
                wPos++ ;
            }
        } else {
            wPos += nNext ;
        }
    }
    // add bias and calculate output
    for(int o = 0 ; o < nNext ; o++ ) {
        nextIn[ o ] += w[ wPos ] ; 
        wPos++ ;
    }
    f->output( nextIn, nextOut, nNext ) ;
}

double *    cNeuralNetwork::_backPropLayer( int layer, double * oError, double learningSpeed ) {
    
    w = weights[ layer ]->getPtr() ; // The weights of the current layer
    
    nFormer = layerSize[ layer ] ;   // # nodes in the former nodes layer
    nNext   = layerSize[ layer + 1 ] ;       // # nodes in the next nodes layer
    
    formerOut = layerOut[ layer ]->getPtr() ; // The output of the former layer
    formerIn  = layerIn[ layer ]->getPtr() ;  // The input of the former layer
    
    cFunction * f = layerFunction[ layer ] ; // Function of the former layer
    
    wPos = 0 ;
    
    iError = new double[ nFormer ] ;   // Error of the input of the former layer
    
    if ( layer > 0 ) {
        
        for(int i = 0 ; i < nFormer ; i++ ) {
            
            iError[ i ] = 0.0 ;
            
            if ( formerOut[ i ] != 0.0 ) {
                
                for (int o = 0 ; o < nNext ; o++ ) {
                    
                    // First get error:
                    iError[ i ] += w[ wPos ] * oError[ o ] ;
                    
                    // Then update the weight:
                    w[ wPos ] += formerOut[ i ] * oError[ o ] ;
                    
                    wPos++ ;
                    
                }
                
            } else {
                
                for (int o = 0 ; o < nNext ; o++ ) {
                    
                    // Only get error:
                    iError[ i ] += w[ wPos ] * oError[ o ] ;
                    
                    wPos++ ;
                    
                }
                
            }
            // Pass the error through the layers function:
			//stringstream s_out;
			//s_out << "iError[" << i << "] = " << iError[ i ] ;
			double deriv = f->derivative( formerIn[ i ], formerOut[ i ] ) ;
            iError[ i ] *= deriv;

			//s_out << "\t\tderivative = " << deriv;
			//s_out << "\t\tnew iError[" << i << "] = " << iError[ i ] ;

			//mp_NN_log->write(s_out.str());
        }
        
    } else {
        
        for(int i = 0 ; i < nFormer ; i++ ) {
            
            if ( formerOut[ i ] != 0.0 ) {
                
                for (int o = 0 ; o < nNext ; o++ ) {
                   
                    // Only update weight:
                    w[ wPos ] += formerOut[ i ] * oError[ o ] ;
                    
                    wPos++ ;
                    
                }
                
            } else {
                
                wPos += nNext ;
                
            }
            
        }
    }

    for (int o = 0 ; o < nNext ; o++ ) {
        
        // Update the bias
        w[ wPos ] += oError[ o ] ;
        
        wPos++ ;
        
    }
    
    
    
    return iError ;
}

void cNeuralNetwork::backPropagate( double *input, double *target, double learningSpeed ) {

    //If the network has been adapted after the last forward propagation,
    //forwardPropagate first to correctly set the hidden activations:
	//cout << "Recently updated: " << recentlyUpdated << endl;
	if ( recentlyUpdated ) {
		forwardPropagate( input ) ;
	} else {
		//Check whether the activations of the layers correspond with the present input
		inputIn = layerIn[ 0 ]->getPtr() ;
		bool useLastActivations = true ;
		for (int i = 0 ; ( i < nInput ) & useLastActivations ; i++ ){
			if ( input[i] != inputIn[i] ) {
				useLastActivations = false ;
			}
		}
		if ( !useLastActivations ) {
			//If the activations don't correspond to the last input (and the network
			//has been adapted in the meantime) set the activations by a forward
			//propagation
			forwardPropagate( input ) ;
		}
	}

    error = new double[ nOutput ] ; //error of the output layer.
    outputOut  = layerOut[ nLayers - 1 ]->getPtr() ;// Output of the output layer
    
    for(int o = 0 ; o < nOutput ; o++) {
        error[ o ] = target[ o ] - outputOut[ o ] ;
    }
    //backPropagate the error
    backPropagateError( input, error, learningSpeed ) ;
    delete[] error ;
}

void        cNeuralNetwork::backPropagateError( double *input, double *error, double learningSpeed ) {
    //If the network has been adapted after the last forward propagation,
    //forwardPropagate first to correctly set the hidden activations:
    if ( recentlyUpdated ) {
        forwardPropagate( input ) ;
    } else {
        //Check whether the activations of the layers correspond with the present input
        inputIn = layerIn[ 0 ]->getPtr() ;
        bool useLastActivations = true ;
        for (int i = 0 ; ( i < nInput ) & useLastActivations ; i++ ){
            if ( input[i] != inputIn[i] ) {
                useLastActivations = false ;
            }
        }
        if ( !useLastActivations ) {
            //If the activations don't correspond to the last input (and the network
            //has been adapted in the meantime) set the activations by a forward
            //propagation
            forwardPropagate( input ) ;
        }
    }
    oError = new double[ nOutput ] ; //error of the output layer.
    
    outputIn   = layerIn[ nLayers - 1 ]->getPtr() ; // Input of the output layer
    outputOut  = layerOut[ nLayers - 1 ]->getPtr() ;// Output of the output layer
    cFunction * f       = layerFunction[ nLayers - 1 ] ;    // Function of the output layer

    //First calculate the error of the input of the output layer:
	//mp_NN_log->write("Writing error of the input of the output layer");
    for( int o = 0 ; o < nOutput ; o++) {
		//stringstream s_out;
        oError[ o ] = learningSpeed*error[ o ] ;
		//s_out << "oError[" << o << "] = " << oError[o];
		double deriv = f->derivative( outputIn[ o ], outputOut[ o ] );
        oError[ o ] *= deriv;
		//s_out << "\t\tderivative = " << deriv;
		//s_out << "\t\tnew oError[" << o << "] = " << oError[o];
		//mp_NN_log->write(s_out.str());
    }
    //mp_NN_log->write("End of error of input of output layer\n");
	//mp_NN_log->write("Propagating error until reaching input layer.");
    //Now propagate until reaching the input layer:
    for (int l = nLayers - 2 ; l >= 0 ; l-- ) {
        iError = _backPropLayer( l, oError, learningSpeed ) ;
        delete [] oError ;
        oError = iError ;
		//mp_NN_log->write("End of layer");
    }
   //mp_NN_log->write("Done propagating.\n");
    delete [] iError ;
    
    recentlyUpdated = true ;
}
    


double *    cNeuralNetwork::forwardPropagate( double *input ) {
    cFunction * f       = layerFunction[ 0 ] ; // Function of the input layer
    
    double * inputIn  = layerIn[ 0 ]->getPtr() ;  // Input of the first layer (to be set)
    double * inputOut = layerOut[ 0 ]->getPtr() ; // Output of the first layer (to be set)
    
    //First set the first layer
    for( int i = 0 ; i < nInput ; i++ ) {
        inputIn[ i ] = input[ i ] ;
    }
    
    f->output( input, inputOut, nInput ) ;
    
    //Now propagate (sets layerIn and layerOut for each layer)
    for ( int l = 0 ; l < (nLayers - 1) ; l++ ) {
        _forwardPropLayer( l ) ;
    }
    
    //Set recentlyUpdated to false to show that the activations where set after the last change to the network
    recentlyUpdated = false ;
    
    return layerOut[ nLayers - 1 ]->getPtr() ; // Output of the last layer

}

double *    cNeuralNetwork::forwardPropagate( vector<double> *input_vector ) {
    if ( (int) input_vector->size() != nInput ) {
        cerr << "Vector input is incorrect size: " << input_vector->size() << " instead of " << nInput << endl ;
        throw -1 ;
    }
    
    double * input = new double[ nInput ] ;
    for ( int i = 0 ; i < nInput ; i++ ) {
        input[ i ] = input_vector->at( i ) ;
    }
    
    double * output = forwardPropagate( input ) ;
    
    delete [] input ;
    
    return output ;
}
    
    
void        cNeuralNetwork::forwardPropagate( double *input, List * output ) {
    cFunction * f       = layerFunction[ 0 ] ; // Function of the input layer
    
    double * inputIn  = layerIn[ 0 ]->getPtr() ;  // Input of the first layer (to be set)
    double * inputOut = layerOut[ 0 ]->getPtr() ; // Output of the first layer (to be set)
    
    //~ cout << "nInput" << nInput << endl ;
    //~ cout << "nOutput" << nOutput << endl ;
    //~ cout << "nLayers" << nLayers << endl ;
    
    //First set the first layer
    for( int i = 0 ; i < nInput ; i++ ) {
        inputIn[ i ] = input[ i ] ;
    }
    
    f->output( input, inputOut, nInput ) ;
    
    //Now propagate (sets layerIn and layerOut for each layer)
    for ( int l = 0 ; l < (nLayers - 1) ; l++ ) {
        _forwardPropLayer( l ) ;
    }

    double * outputOut = layerOut[ nLayers - 1 ]->getPtr() ; // Output of the last layer
    
    //Finally, return the output of the last layer through the argument
    output->contents = new double[ nOutput ] ;
    for ( int o = 0 ; o < nOutput ; o++ ) {
        output->contents[ o ] = outputOut[ o ] ;
    }
    output->length = nOutput ;
    //Set recentlyUpdated to false to show that the activations where set after the last change to the network
    recentlyUpdated = false ;
}

double      cNeuralNetwork::getActivation( int layer, int node )  {
    return layerOut[ layer ]->get( node ) ;
}

void        cNeuralNetwork::getActivations( int layer, List * activations )  {
    double * layerActivation = layerOut[ layer ]->getPtr() ; // Output of the requested layer
    
    //Return the output of the requested layer through the argument
    int nActivations = layerSize[ layer ] ;
    activations->contents = new double[ nActivations ] ;
    for ( int o = 0 ; o < nActivations ; o++ ) {
        activations->contents[ o ] = layerActivation[ o ] ;
    }
    activations->length = nActivations ;
}

double *    cNeuralNetwork::getActivations( int layer )  {
    return layerOut[ layer ]->getPtr() ; // Output of the requested layer
}

double      cNeuralNetwork::getWeights( int layer, int i, int j )  {
    return weights[ layer ]->get( i*(layerSize[ layer + 1 ]) + j ) ;
}

void        cNeuralNetwork::setWeights( int layer, int i, int j, double val ) {
    recentlyUpdated = true ;
	//weights[ layer ]->set( i*(layerSize[ layer + 1 ]) + j, val ) ; //HIER
	if(j == 0)
		weights[ layer ]->set( i, val ) ;
	else
		weights[ layer ]->set( i, j, val ) ; 
}

void        cNeuralNetwork::randomizeWeights( double min, double max ) {
    for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
            for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
				#ifdef WIN32
					setWeights( l, i, o, min + (max - min)* double(rand())/RAND_MAX) ;
				#else
					setWeights( l, i, o, min + (max - min)*drand48() ) ;
				#endif
            }
        }
    }
}

void        cNeuralNetwork::randomizeWeights( double min, double max, int seed ) {
    srand( seed ) ;
    for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
            for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
                
					#ifdef WIN32
						setWeights( l, i, o, min + (max - min)* double(rand())/RAND_MAX) ;
					#else
						setWeights( l, i, o, min + (max - min)*drand48() ) ;
					#endif
            }
        }
    }
}

void        cNeuralNetwork::printNetwork() {
    for ( int l = 0 ; l < (nLayers - 1) ; l++ ) {
        _printLayer( l ) ;
    }
}

void        cNeuralNetwork::_printLayer( int layer ) {
    double * w = weights[ layer ]->getPtr() ; // The weights of the current layer
    cout << "layer " << layer ;
    
    int nFormer = layerSize[ layer ] ;       // # nodes in the former nodes layer
    int nNext   = layerSize[ layer + 1 ] ;   // # nodes in the next nodes layer
    
    cout << " nFormer " << nFormer ;
    cout << " nNext " << nNext << endl ;
    
    int wPos = 0 ;
    for( int i = 0 ; i < nFormer ; i++ ) {
        for( int o = 0 ; o < nNext ; o++ ) {
            cout << w[ wPos ] << ", " ;
            wPos++ ;
        }
        cout << '\n' ;
    }
    // add bias and calculate output
    for( int o = 0 ; o < nNext ; o++ ) {
        cout << " B " <<  w[ wPos ] << '\n' ; 
        wPos++ ;
    }
}

int         cNeuralNetwork::read_int(istream &is) {
    int result;
    //char *s = (char *) &result;
    //is.read(s, sizeof(result));
	is >> result;
    return result;

}

void        cNeuralNetwork::write_int(ostream &os, int result) {
    //char *s = (char *) &result;
    //os.write(s, sizeof(result));
	os << setw(15);
	os << result;
}

double      cNeuralNetwork::read_double(istream &is) {
    double result;
    //char *s = (char *) &result;
    //is.read(s, sizeof(result));
	is >> result;
    return result;
}

void        cNeuralNetwork::write_double(ostream &os, double result) {
    //char *s = (char *) &result;
    //os.write(s, sizeof(result));
	os << setw(15);
	os << result;
}

void cNeuralNetwork::readNetwork( const char * file ) { 
    ifstream ifile ;
    
    ifile.open(file, ifstream::in) ;
	if(ifile.is_open()){
		nLayers                 = read_int( ifile ) ; 
    
		int * layerSizeInit     = new int[nLayers] ;
		int * layerFunctionInit = new int[nLayers] ;
    
		for ( int l = 0 ; l < nLayers ; l++ ) {
			layerSizeInit[l]        = read_int( ifile ) ; 
			layerFunctionInit[l]    = read_int( ifile ) ;
		}

		nInput  = layerSizeInit[ 0 ] ;
		nOutput = layerSizeInit[ nLayers - 1 ] ;
		//~ cout << "nInput" << nInput << endl ;
		//~ cout << "nOutput" << nOutput << endl ;
		//~ cout << "nLayers" << nLayers << endl ;
    
		layerFunction = new cFunction*[ nLayers ] ;
		layerFunctionInts   = new int[ nLayers ] ;
		weights       = new Matrix*[ nLayers - 1 ] ; // # weights layers = # node layers - 1
		layerIn       = new Matrix*[ nLayers ] ; 
		layerOut      = new Matrix*[ nLayers ] ;
		layerSize     = new int[ nLayers ] ;
    
		//klopt deze loop wel??
		for( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			weights[ l ]  = new Matrix( layerSizeInit[ l ] + 1, layerSizeInit[ l  + 1]) ; // layerSize[ l ] + 1, for bias
		}
		//
		for( int l = 0 ; l < nLayers ; l++ ) {
			layerSize[ l ] = layerSizeInit[ l ] ;
			layerIn[ l ]  = new Matrix( layerSize[ l ] ) ;
			layerOut[ l ] = new Matrix( layerSize[ l ] ) ;
            
			layerFunctionInts[ l ] = layerFunctionInit[ l ] ;
			if ( layerFunctionInit[ l ] == TANH ) {
				layerFunction[ l ] = new cTanH() ;
			} else if ( layerFunctionInit[ l ] == LINEAR ) {
				layerFunction[ l ] = new cLinear() ;
			} else {
				cout << "WARNING: Unknown layer function type: " ;
				cout << layerFunctionInit[ l ] << '\n' ;
				cout << "layer: " << l << '\n' ;
				ifile.close();
				#ifdef WIN32
					char end;
					cin>>end;
				#endif
				exit(-2) ;
			}
		}
		// Get weights
		for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
				for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
					setWeights( l, i, o, read_double( ifile ) ) ;
				}
			}
		}
		recentlyUpdated = true ;
    
		delete [] layerSizeInit ;
		delete [] layerFunctionInit ;
		ifile.close();
	} else
		cerr << "Could not open neural network file" << endl;
}


void cNeuralNetwork::readNetwork( string file ) { 
    ifstream ifile ;
    
    ifile.open( file.c_str(), ifstream::in ) ;
	if(ifile.is_open()){
		nLayers                 = read_int( ifile ) ; 
    
		int * layerSizeInit     = new int[nLayers] ;
		int * layerFunctionInit = new int[nLayers] ;
    
		for ( int l = 0 ; l < nLayers ; l++ ) {
			layerSizeInit[l]        = read_int( ifile ) ; 
			layerFunctionInit[l]    = read_int( ifile ) ;
		}

		nInput  = layerSizeInit[ 0 ] ;
		nOutput = layerSizeInit[ nLayers - 1 ] ;
		//~ cout << "nInput" << nInput << endl ;
		//~ cout << "nOutput" << nOutput << endl ;
		//~ cout << "nLayers" << nLayers << endl ;
    
		layerFunction = new cFunction*[ nLayers ] ;
		layerFunctionInts   = new int[ nLayers ] ;
		weights       = new Matrix*[ nLayers - 1 ] ; // # weights layers = # node layers - 1
		layerIn       = new Matrix*[ nLayers ] ; 
		layerOut      = new Matrix*[ nLayers ] ;
		layerSize     = new int[ nLayers ] ;
    
		for( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			weights[ l ]  = new Matrix( layerSizeInit[ l ] + 1, layerSizeInit[ l  + 1]) ; // layerSize[ l ] + 1, for bias
		}
		for( int l = 0 ; l < nLayers ; l++ ) {
			layerSize[ l ] = layerSizeInit[ l ] ;
			layerIn[ l ]  = new Matrix( layerSize[ l ] ) ;
			layerOut[ l ] = new Matrix( layerSize[ l ] ) ;
            
			layerFunctionInts[ l ] = layerFunctionInit[ l ] ;
			if ( layerFunctionInit[ l ] == TANH ) {
				layerFunction[ l ] = new cTanH() ;
			} else if ( layerFunctionInit[ l ] == LINEAR ) {
				layerFunction[ l ] = new cLinear() ;
			} else {
				cout << "WARNING: Unknown layer function type: " ;
				cout << layerFunctionInit[ l ] << '\n' ;
				cout << "layer: " << l << '\n' ;
				ifile.close();
				#ifdef WIN32
					char end;
					cin>>end;
				#endif
				exit(-2) ;
			}
		}
		cout << " Getting weights from file: " << file << endl;
		// Get weights
		for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			//cout << "Layer " << l << ":\n";
			for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
				//cout << "input: " << i << endl;
				for ( int o = 0 ; o < layerSize[ l + 1]  ; o++ ) {
					//cout << "\toutput: " << o << ". "; 
					//cout << "weight: " << weight <<endl;
					setWeights( l, i, o, read_double( ifile )  ) ;
				}
			}
		}
		recentlyUpdated = true ;
    
		delete [] layerSizeInit ;
		delete [] layerFunctionInit ;
		ifile.close();
	} else
		cerr << "Could not open neural network file" << endl;
}


void cNeuralNetwork::readNetwork(ifstream& ifile) { 

	if(ifile.is_open()){
		nLayers                 = read_int( ifile ) ; 
    
		int * layerSizeInit     = new int[nLayers] ;
		int * layerFunctionInit = new int[nLayers] ;
    
		for ( int l = 0 ; l < nLayers ; l++ ) {
			layerSizeInit[l]        = read_int( ifile ) ; 
			layerFunctionInit[l]    = read_int( ifile ) ;
		}

		nInput  = layerSizeInit[ 0 ] ;
		nOutput = layerSizeInit[ nLayers - 1 ] ;
		//~ cout << "nInput" << nInput << endl ;
		//~ cout << "nOutput" << nOutput << endl ;
		//~ cout << "nLayers" << nLayers << endl ;
    
		layerFunction = new cFunction*[ nLayers ] ;
		layerFunctionInts   = new int[ nLayers ] ;
		weights       = new Matrix*[ nLayers - 1 ] ; // # weights layers = # node layers - 1
		layerIn       = new Matrix*[ nLayers ] ; 
		layerOut      = new Matrix*[ nLayers ] ;
		layerSize     = new int[ nLayers ] ;
    
		for( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			weights[ l ]  = new Matrix( layerSizeInit[ l ] + 1, layerSizeInit[ l + 1] ) ; // layerSize[ l ] + 1, for bias
		}
		for( int l = 0 ; l < nLayers ; l++ ) {
			layerSize[ l ] = layerSizeInit[ l ] ;
			layerIn[ l ]  = new Matrix( layerSize[ l ] ) ;
			layerOut[ l ] = new Matrix( layerSize[ l ] ) ;
            
			layerFunctionInts[ l ] = layerFunctionInit[ l ] ;
			if ( layerFunctionInit[ l ] == TANH ) {
				layerFunction[ l ] = new cTanH() ;
			} else if ( layerFunctionInit[ l ] == LINEAR ) {
				layerFunction[ l ] = new cLinear() ;
			} else {
				cout << "WARNING: Unknown layer function type: " ;
				cout << layerFunctionInit[ l ] << '\n' ;
				cout << "layer: " << l << '\n' ;
				ifile.close();
				#ifdef WIN32
						char end;
						cin>>end;
				#endif
				exit(-2) ;
			}
		}
		// Get weights
		for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
				for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
					setWeights( l, i, o, read_double( ifile ) ) ;
				}
			}
		}
		recentlyUpdated = true ;
    
		delete [] layerSizeInit ;
		delete [] layerFunctionInit ;
		//ifile.close();
	} else
		cerr << "Could not open neural network file" << endl;
}
    
void cNeuralNetwork::writeNetwork( std::string file ) { 
    ofstream ofile ;
    
    ofile.open(file.c_str(), ofstream::trunc ) ;
    if(ofile.is_open())
	{
		write_int( ofile, nLayers ) ; 
    
		for ( int l = 0 ; l < nLayers ; l++ ) {
			write_int( ofile, layerSize[l] ) ; 
			write_int( ofile, layerFunctionInts[l] ) ;
		}
		ofile << "\n";
		// Weights
		for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
				for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
					write_double( ofile, getWeights( l, i, o ) ) ;
				}
			}
			ofile << "\n";
		}
		ofile.close();
	}else {
		cerr << "Can't open file "<< file << "!\n";
		throw std::invalid_argument("Can't open specified file");
	}
}

    
void cNeuralNetwork::writeNetwork( const char * file ) { 
    ofstream ofile ;
    
    ofile.open( file, ofstream::trunc ) ;
    if(ofile.is_open())
	{
		write_int( ofile, nLayers ) ; 
    
		for ( int l = 0 ; l < nLayers ; l++ ) {
			write_int( ofile, layerSize[l] ) ; 
			write_int( ofile, layerFunctionInts[l] ) ;
		}

		// Weights
		for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
				for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
					write_double( ofile, getWeights( l, i, o ) ) ;
				}
			}
		}
		ofile.close();
	}else {
		cerr << "Can't open file "<< file << "!\n";
		throw std::invalid_argument("Can't open specified file");
	}
}
   
void cNeuralNetwork::writeNetwork( ofstream& ofile) { 
    if(ofile.is_open())
	{
		write_int( ofile, nLayers ) ; 
    
		for ( int l = 0 ; l < nLayers ; l++ ) {
			write_int( ofile, layerSize[l] ) ; 
			write_int( ofile, layerFunctionInts[l] ) ;
		}

		// Weights
		for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
			for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
				for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
					write_double( ofile, getWeights( l, i, o ) ) ;
				}
			}
		}
		ofile.close();
	}else {
		cerr << "Please provide an open ofstream!\n";
		throw std::invalid_argument("provided ofstream should be open");
	}
}

