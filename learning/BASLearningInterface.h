#ifndef BASLEARNING_INTERFACE_H
#define BASLEARNING_INTERFACE_H

#include "RLInterface.h"
#include "BinaryActionSearch.h"

class BASLearningInterface: public RLInterface
{
	public:

		//initialisation
		BASLearningInterface(void);
		~BASLearningInterface(void);
		void init();
		void init(const bool& automatic);
		void init(const bool& automatic, const char* nn_filename);

		//learning functions
		bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

		//other
		void writeNetwork(int identifier, int step);

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