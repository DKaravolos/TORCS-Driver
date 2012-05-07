#ifndef LEARNING_INTERFACE_H
#define LEARNING_INTERFACE_H

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "..\rlcpp\Action.h"
#include "..\rlcpp\World.h"
#include "..\rlcpp\StateActionAlgorithm.h"
#include "..\rlcpp\Qlearning.h"
#include "..\rlcpp\State.h"
#include "..\rlcpp\Experiment.h"
#include "..\rlcpp\StateActionUtils.h"

#include "TorcsWorld.h"
#include "StateActionMemory.h"
#include "ExperimentParameters.h"
//#include "..\utilities\writeToFile.h"
#include "..\utilities\Writer.h"


class LearningInterface
{
	public:
		enum UpdateOption{RANDOM,TD};

		LearningInterface(void);
		~LearningInterface(void);
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

		//state functions
		void setState(std::vector<double>* features); //called from TORCS before mainloop to set state for learning algorithm
		void printState();
	
		//world functions
		//void computeReward(int new_dist);
		inline double getReward()	{ return m_reward;}
		inline void setEOE(bool eoe){ mp_world->setEOE(eoe);}
		inline bool getEOE(){ return mp_world->endOfEpisode();}

		//other
		bool learningUpdateStep(); //called from TORCS to do learning. Returns whether experiment is over or not.
		bool learningUpdateStep(bool store_tuples);
		void updateWithOldTuple(UpdateOption option);

	protected:
		//datamembers
		TorcsWorld* mp_world;
		//StateActionAlgorithm* mp_algorithm;
		Qlearning* mp_algorithm;
		Experiment* mp_experiment;
		State* mp_prev_state;
		State* mp_current_state;
		Action* mp_prev_action;
		Action* mp_current_action;
		ExperimentParameters* mp_parameters;
		double* mp_torcs_action;

		double m_reward;

		Writer* mp_log;
		StateActionMemory* mp_memory;

		//functions:
			//init
		void initState();
		void initActions();
		void initExperimentParam();

			//other
		//void explore(State*, Action*);
		//inline void worldSetAction() {dp_action = dp_world->getAction();}
};

#endif /*LEARNING_INTERFACE_H*/