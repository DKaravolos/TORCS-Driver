#ifndef QOS_LEARNING_INTERFACE_H
#define QOS_LEARNING_INTERFACE_H

#include "RLInterface.h"
#include "../rlcpp/StateActionAlgorithm.h"
#include "../rlcpp/Qlearning.h"

class QOSLearningInterface: public RLInterface
{
	public:

		QOSLearningInterface(void);
		~QOSLearningInterface(void);
		void init();
		void init(const bool& automatic_experiment);
		void init(const bool& automatic_experiment, const char* nn_filename);

		//other
		bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);
		void writeNetwork(int identifier, int steps);

	protected:
		//datamembers
		Qlearning* mp_algorithm;

	private:
		//functions:
		void _init(const bool& automatic_experiment);
		void initState();
		void initActions();
};

#endif /*QOS_LEARNING_INTERFACE_H*/