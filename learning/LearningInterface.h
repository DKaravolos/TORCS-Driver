#ifndef LEARNING_INTERFACE_H
#define LEARNING_INTERFACE_H

#include <vector>
#include <iostream>

#include "..\..\rlcpp\State.h"

class LearningInterface
{
public:
	LearningInterface(void);
	~LearningInterface(void);
	//init functions
	

	//other
	void setState(std::vector<double>* features);
	void printState();

private:
	//datamembers
	State* dp_current_state;
	
	//functions:
	void initState();
};

#endif /*LEARNING_INTERFACE_H*/