#ifndef LEARNING_INTERFACE_H
#define LEARNING_INTERFACE_H

#include "RLInterface.h"
#include "..\rlcpp\StateActionAlgorithm.h"
#include "..\rlcpp\Qlearning.h"

class LearningInterface: public RLInterface
{
	public:
		//initialisation
		LearningInterface(void);
		~LearningInterface(void);
		virtual void init();
		virtual void init(const char* nn_filename);

		//learning functions
		virtual bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

		//other
		void writeNetwork(int identifier);

	protected:
		//datamembers
		Qlearning* mp_algorithm;

		//functions:
		virtual void initState();
		virtual void initActions();
};

#endif /*LEARNING_INTERFACE_H*/
