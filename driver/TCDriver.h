#ifndef TCDriver_H_
#define TCDriver_H_

#include <iostream>
#include <fstream>
#include <sstream>
//
#include <vector>
//Functions/classes by Daniel:
#include "RLDriver.h"
#include "../learning/TCLearningInterface.h"
#include "../utilities/Writer.h"

using namespace std;

class TCDriver : public RLDriver
{

public:
	
	// Constructor
	TCDriver();
	TCDriver(const string& log_dir, const int& steps, const int& runs, const bool& save_data);

	// SimpleDriver implements a simple and heuristic controller for driving
	//virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	virtual void onShutdown();
	
	// Print a restart message 
	//virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	//virtual void init(float *angles);

private:
	
	////////Functions added by Daniel:
	virtual void initInterface(const bool& load_network, const bool& m_automatic_experiment);
	void askLoadNetwork();

};

#endif /*TCDriver_H_*/
