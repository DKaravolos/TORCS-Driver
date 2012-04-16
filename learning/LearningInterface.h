#ifndef LEARNING_INTERFACE_H
#define LEARNING_INTERFACE_H

#include <vector>
#include <iostream>
#include <fstream>
#include "..\rlcpp\Action.h"
#include "..\rlcpp\World.h"
#include "..\rlcpp\Algorithm.h"
#include "..\rlcpp\Qlearning.h"
#include "..\rlcpp\State.h"
#include "..\rlcpp\Experiment.h"
#include "..\learning\TorcsWorld.h"
//#include "ExperimentParameters.h"



class LearningInterface
{

public:
	LearningInterface(void);
	~LearningInterface(void);

	//Loop of a time step:
	/*
	- setRewardPrevAction()
	- setState()
	- experimentMainloop()
	- getAction()
	*/

	//driver functions
	double* getAction(); //called from TORCS to receive computed action
	void setRewardPrevAction(int distance); //called from TORCS before mainloop to get reward of previous action

	//state functions
	void setState(std::vector<double>* features); //called from TORCS before mainloop to set state for learning algorithm
	void printState();
	
	//world functions
	//void computeReward(int new_dist);
	inline double getReward() { return m_reward;}

	//other
	double* experimentMainLoop(); //called from TORCS to do learning

private:
	//datamembers
	TorcsWorld* mp_world;
	Algorithm* mp_algorithm;
	Experiment* mp_experiment;
	State* mp_current_state;
	Action* mp_prev_action;
	Action* mp_current_action;

	int m_reward;

	//functions:
		//init
	void initState();
	void initActions();

		//other
	void explore(State*, Action*);
	//inline void worldSetAction() {dp_action = dp_world->getAction();}
};

#endif /*LEARNING_INTERFACE_H*/