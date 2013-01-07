#ifndef TCLEARNING_INTERFACE_H
#define TCLEARNING_INTERFACE_H

#include "..\learning\RLInterface.h"
#include "..\rlcpp\StateActionAlgorithm.h"
#include "..\learning\TileCodingSmall.h"
#include "..\learning\TileCodingHM.h"
#include <string>

#include "..\driver\CarControl.h"
#include "..\driver\CarState.h"

class TCLearningInterface: public RLInterface
{
	public:
		//initialisation
		TCLearningInterface(const std::string& log_dir);
		~TCLearningInterface(void);
		void init();
		void init(const bool& automatic_experiment);
		void init(const bool& automatic_experiment, const char* qtable_filename);

		//learning functions
		bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

		//other
		//TileCodingHM* getAlgorithm(){return mp_algorithm;}
		void doInformedAction(Action* action);
		void writeNetwork(int identifier, int step); //only for inheritance (calls writeQTable)
		void writeQTable(int identifier, int step);

		//for automatic experiment
		Experiment* getExperiment(){return mp_experiment;}
		CarControl* public_car_control;
		CarState* public_car_state;
	protected:
		//datamembers
		TileCodingHM* mp_algorithm;

		//functions:
		void _init(const bool& auto_exp);
		virtual void initState();
		virtual void initActions();
		void loadQTable(int id, int step);
};

#endif /*TCLEARNING_INTERFACE_H*/
