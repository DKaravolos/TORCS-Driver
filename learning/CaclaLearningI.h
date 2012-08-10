#ifndef CALCA_LEARNING_INTERFACE_H
#define CALCA_LEARNING_INTERFACE_H

#include "RLInterface.h"
#include "../rlcpp/Cacla.h"


class CaclaLearningI: public RLInterface
{
	public:
		//initialisation
		CaclaLearningI(void);
		~CaclaLearningI(void);
		virtual void init();
		virtual void init(const char* ann_filename, const char* vnn_filename);
		void init(const char* nn_filename); //added only for inheritance purposes; do not use.

		//learning functions
		bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

		//other
		void writeNetwork(int identifier);

	protected:
		//datamembers
		Cacla* mp_algorithm;

		//functions:
			//init
		virtual void initState();
		virtual void initActions();
};

#endif /*CALCA_LEARNING_INTERFACE_H*/