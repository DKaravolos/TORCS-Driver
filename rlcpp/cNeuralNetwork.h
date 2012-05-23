#ifndef CNEURALNETWORK_H
#define CNEURALNETWORK_H
# include <vector>
# include <iostream>
# include <iomanip> //for writing to file
#include <sstream>
# include <fstream>
# include <string>
# include "cFunction.h"
# include "cTanH.h"
# include "cLinear.h"
# include "Matrix.h"
# include "List.h"
# include "cThreshold.h"

#include "..\utilities\Writer.h"
using namespace std;

class cNeuralNetwork {
    public:
        cNeuralNetwork( const char * );
		cNeuralNetwork( std::string);
		cNeuralNetwork( ifstream& is);
        cNeuralNetwork( int, int * );
        cNeuralNetwork( int, int *, int * );
        ~cNeuralNetwork();
    
        void init( int, int *, int * ) ;
    
        void forwardPropagate( double *, List * );
        double * forwardPropagate( double * );
        double * forwardPropagate( vector<double> * );
        
        void backPropagate(      double *, double *, double );
        void backPropagateError( double *, double *, double );
    
        void changeFunction( int, int ) ;
    
        double getActivation( int, int ) ;
        void getActivations( int, List * ) ;
        double * getActivations( int ) ;
        double getWeights( int, int, int ) ;
        void setWeights( int, int, int, double ) ;
        void randomizeWeights( double, double ) ;
        void randomizeWeights( double, double, int ) ;
        void printNetwork() ;
        void readNetwork (const char *);
		void readNetwork (std::string);
		void readNetwork (ifstream& is);
        void writeNetwork( const char * ) ;
		void writeNetwork( std::string ) ;
		void writeNetwork( ofstream& ofile);

    private:
		//datamembers:
        int nLayers, nInput, nOutput, nFormer, nNext ;
        unsigned int wPos ;
        int * layerSize ;
        int * layerFunctionInts ;
        double * formerOut ;
        double * formerIn ;
        double * nextIn ;
        double * nextOut ;
        double * iError ;
        double * oError ;
        double * w ;
        double * inputIn ;
        double * inputOut ;
        double * outputIn ;
        double * outputOut ;
        double * error ;
        cFunction ** layerFunction ;
        Matrix ** weights ;
        Matrix ** layerIn ;
        Matrix ** layerOut ;
       // int i, h, o, l ; //Indices
		Writer* mp_NN_log;

		//functions:
		void pointerInit();
		void _forwardPropLayer( int ) ;
        double * _backPropLayer( int layer, double * outError, double learningSpeed ) ;
        bool recentlyUpdated ;
        void _printLayer( int ) ;
        int read_int( istream & ) ;
        void write_int( ostream &, int ) ;
        double read_double( istream & ) ;
        void write_double( ostream &, double ) ;

};

#endif // CNEURALNETWORK_H
