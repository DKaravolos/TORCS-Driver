#ifndef TCLEARNING_INTERFACE_H
#define TCLEARNING_INTERFACE_H

#include "..\learning\RLInterface.h"
#include "..\rlcpp\StateActionAlgorithm.h"
#include "..\learning\TileCoding.h"
#include <string>

class TCLearningInterface: public RLInterface
{
	public:
		//initialisation
		TCLearningInterface(void);
		~TCLearningInterface(void);
		virtual void init();
		virtual void init(const char* qtable_filename);

		//learning functions
		virtual bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

		//other
		void writeNetwork(int identifier, int step); //only for inheritance (calls writeQTable)
		void writeQTable(int identifier, int step);

	//protected:
		//datamembers
		TileCoding* mp_algorithm;

		//functions:
		void _init();
		virtual void initState();
		virtual void initActions();
		void loadQTable(int id, int step);
};

#endif /*TCLEARNING_INTERFACE_H*/
