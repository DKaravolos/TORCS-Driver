#ifndef LEARNING_INTERFACE_H
#define LEARNING_INTERFACE_H

#include <vector>
#include <iostream>
#include <fstream>
#include "..\..\rlcpp\Action.h"
#include "..\..\rlcpp\World.h"
#include "..\..\rlcpp\Algorithm.h"
#include "..\..\rlcpp\Qlearning.h"
#include "..\..\rlcpp\State.h"
#include "..\learning\TorcsWorld.h"
#include "ExperimentParameters.h"

class LearningInterface
{

public:
	LearningInterface(void);
	~LearningInterface(void);
	//driver functions
	vector<double> getAction(); //returns the action outputs to TORCS
	
	//state functions
	void setState(std::vector<double>* features);
	void printState();
	
	//world functions
	void computeReward(int new_dist);
	inline double getReward() { return d_reward;}

	//other
	double* experimentMainLoop();

private:
	//datamembers
	State* dp_current_state;
	Algorithm* dp_algorithm;
	TorcsWorld* dp_world;
	ExperimentParameters d_param;

	int d_last_dist;
	double d_reward;

	//functions:
		//init
	void initState();

		//other
	void explore(State*, Action*);
	//inline void worldSetAction() {dp_action = dp_world->getAction();}
};

#endif /*LEARNING_INTERFACE_H*/