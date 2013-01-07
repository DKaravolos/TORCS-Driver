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

		//Constructor, Destructor & init functions
		RLInterface(void);
		~RLInterface(void);
		virtual void init() = 0;
		virtual void init(const bool& automatic) = 0;
		virtual void init(const bool& automatic, const char* nn_filename) = 0;
		//Loop of a time step:
		/*
		- setRewardPrevAction()
		- setState()
		- experimentMainloop()
		- getAction()
		*/

		//Driver functions
		double* getAction(); //called from TORCS to receive computed action
		void setRewardPrevAction(double reward); //called from TORCS before mainloop to get reward of previous action

		//state of the world functions
		inline bool getEOE()		{return mp_world->endOfEpisode();}
		inline double getReward()	{return m_reward;}
		inline int getSteps()		{return mp_parameters->step;}
		void setState(std::vector<double>* features); //called from TORCS before mainloop to set state for learning algorithm
		void setEOE();
		void setFirstTime(bool);

		//log functions
		void printState();
		void logState(int timestamp);
		void logAction(int timestamp);
		virtual void writeNetwork(int identifier, int steps) = 0;

		//other
		virtual bool learningUpdateStep(); //called from TORCS to do learning. Returns whether experiment is over or not.
		virtual bool learningUpdateStep(bool store_tuples, UpdateOption option) = 0;
		virtual void updateWithOldTuple(UpdateOption option) =0;

		//Functions for automatic experiments
		void changeLogWriterTo(std::string& new_file);
		void changeRewardWriterTo(std::string& new_file);
		virtual Experiment* getExperiment(){return mp_experiment;}
		inline void setEta(double eta) {m_eta = eta;}

	protected:
		//datamembers for settings
		TorcsWorld* mp_world;
		Experiment* mp_experiment;
		ExperimentParameters* mp_parameters;

		//datamembers for RL
		State* mp_prev_state;
		State* mp_current_state;
		Action* mp_prev_action;
		Action* mp_current_action;
		double m_reward;
		double m_eta; // percentage of times that a heuristic action is picked.

		//datamembers for comm. with TORCS
		double* mp_torcs_action;

		//datamembers for log
		std::string m_log_dir;
		Writer* mp_log;
		Writer* mp_reward_log;
		StateActionMemory* mp_memory;

		//user preferences
		bool m_explore;
		bool m_update;

		//functions:
			//init
		//void initState(); //cannot be RLInterface function, because it niets mp_algorithm
		//void initActions();//cannot be RLInterface function, because it niets mp_algorithm
		void initExperimentParam();
		void askExplore();
		void askUpdate();
};

#endif /*RL_INTERFACE_H*/
