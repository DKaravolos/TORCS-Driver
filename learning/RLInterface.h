#ifndef RL_INTERFACE_H
#define RL_INTERFACE_H

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "..\rlcpp\Action.h"
#include "..\rlcpp\World.h"
#include "..\rlcpp\State.h"
#include "..\rlcpp\Experiment.h"
#include "..\rlcpp\StateActionUtils.h"

#include "TorcsWorld.h"
#include "StateActionMemory.h"
#include "ExperimentParameters.h"
#include "..\utilities\Writer.h"


class RLInterface
{
	public:
		enum UpdateOption{RANDOM,TD};

		RLInterface(void);
		~RLInterface(void);
		virtual void init() = 0;
		virtual void init(const char* nn_filename) = 0;
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
		virtual void writeNetwork(int identifier, int steps) = 0;
		void setFirstTime(bool);

		//other
		virtual bool learningUpdateStep(); //called from TORCS to do learning. Returns whether experiment is over or not.
		virtual bool learningUpdateStep(bool store_tuples, UpdateOption option) = 0;
		virtual void updateWithOldTuple(UpdateOption option) =0;

	protected:
		//datamembers
		TorcsWorld* mp_world;
		//Qlearning* mp_algorithm;
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

		//user preferences
		bool m_explore;

		//functions:
			//init
		virtual void initState() =0;
		virtual void initActions() =0;
		void initExperimentParam();
		void askExplore();

			//other
};

#endif /*RL_INTERFACE_H*/
