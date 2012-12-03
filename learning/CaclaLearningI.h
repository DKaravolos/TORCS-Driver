#ifndef CALCA_LEARNING_INTERFACE_H
#define CALCA_LEARNING_INTERFACE_H

#include <stdexcept>
#include "RLInterface.h"
#include "../rlcpp/Cacla.h"


class CaclaLearningI: public RLInterface
{
	public:
		//initialisation
		CaclaLearningI(void);
		~CaclaLearningI(void);
		virtual void init();
		virtual void init(const bool& automatic);
		void init(const bool& automatic, const char* nn_filename); //added only for inheritance purposes; do not use.
		virtual void init(const bool& automatic, const char* ann_filename, const char* vnn_filename);

		//learning functions
		bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

		//other
		void writeNetwork(int identifier, int step);

	protected:
		//datamembers
		Cacla* mp_algorithm;

		//functions:
			//init
		void _init(const bool& automatic);
		virtual void initState();
		virtual void initActions();
};

#endif /*CALCA_LEARNING_INTERFACE_H*/
