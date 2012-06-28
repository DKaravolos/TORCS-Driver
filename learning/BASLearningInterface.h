#ifndef BASLEARNING_INTERFACE_H
#define BASLEARNING_INTERFACE_H

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "..\rlcpp\Action.h"
#include "..\rlcpp\World.h"
#include "BinaryActionSearch.h"
#include "BASWithRoots.h"
#include "..\rlcpp\State.h"
#include "..\rlcpp\Experiment.h"
#include "..\rlcpp\StateActionUtils.h"

#include "TorcsWorld.h"
#include "StateActionMemory.h"
#include "ExperimentParameters.h"
#include "..\utilities\Writer.h"


class BASLearningInterface
{
	public:
		enum UpdateOption{RANDOM,TD};

		BASLearningInterface(void);
		~BASLearningInterface(void);
		void init();
		void init(const char* nn_filename);
		//Loop of a time step:
		/*
		- setRewardPrevAction()
		- setState()
		- experimentMainloop()
		- getAction()
		*/

		//driver functions
		double* getAction(); //called from TORCS to receive computed action
		void setRewardPrevAction(double reward); //called from TORCS before mainloop to get reward of previous action

		//state of the world functions
		void setState(std::vector<double>* features); //called from TORCS before mainloop to set state for learning algorithm
		void printState();
		void logState(int timestamp);
		void logAction(int timestamp);
		inline double getReward()	{ return m_reward;}
		void setEOE();
		inline bool getEOE(){ return mp_world->endOfEpisode();}

		//other
		bool learningUpdateStep(); //called from TORCS to do learning. Returns whether experiment is over or not.
		bool learningUpdateStep(bool store_tuples, UpdateOption option);
		void updateWithOldTuple(UpdateOption option);

	protected:
		//datamembers
		TorcsWorld* mp_world;
		//BASWithRoots* mp_algorithm;
		BinaryActionSearch* mp_algorithm;
		Experiment* mp_experiment;
		State* mp_prev_state;
		State* mp_current_state;
		Action* mp_prev_action;
		Action* mp_current_action;
		ExperimentParameters* mp_parameters;
		double* mp_torcs_action;

		double m_reward;

		Writer* mp_log;
		Writer* mp_reward_log;
		StateActionMemory* mp_memory;

		//functions:
			//init
		void initState();
		void initActions();
		void initExperimentParam();

			//other
};

#endif /*BASLEARNING_INTERFACE_H*/