#ifndef BASLEARNING_INTERFACE_H
#define BASLEARNING_INTERFACE_H

#include "RLInterface.h"
#include "BinaryActionSearch.h"

class BASLearningInterface: public RLInterface
{
	public:

		BASLearningInterface(void);
		~BASLearningInterface(void);
		void init();
		void init(const char* nn_filename);

		//learning functions
		bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

	protected:
		//datamembers
		//BASWithRoots* mp_algorithm;
		BinaryActionSearch* mp_algorithm;

		//functions:
			//init
		void initState();
		void initActions();
};

#endif /*BASLEARNING_INTERFACE_H*/