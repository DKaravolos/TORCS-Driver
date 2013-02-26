#ifndef BASWITHROOTS_H
#define BASWITHROOTS_H

#include "BinaryActionSearch.h"
#include "./../rlcpp/cNeuralNetwork.h"

class BASWithRoots: public BinaryActionSearch
{
	public:
		BASWithRoots	(const char * parameterFile, World * w);
		BASWithRoots	(const char * parameterFile, World * w, string QNN_file);
		~BASWithRoots	(void);

		//Max Action functions
		virtual void getMaxAction	(State * state, Action * action);
		void withRootsGetMax(State * state, Action * action);

		//Update functions
		virtual void update	(State * st, Action * action, double rt, State * st_,
							 bool endOfEpisode, double * learningRate, double gamma);

		//NN functions
		virtual void writeQNN(std::string QNN_file);

	private:
		//Update functions
		//virtual void updateAugmentedStates(double* NN_input, int action, double learningRate);

		//NN functions
		virtual void readQNN(std::string QNN_file);
		void initQStay(std::string QNN_file);

		//Neural Network
		vector<cNeuralNetwork*> QNN_stay;
		
};

#endif // BASWITHROOTS_H