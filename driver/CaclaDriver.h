#ifndef CaclaDriver_H_
#define CaclaDriver_H_

//Functions/classes by Daniel:
#include "RLDriver.h"
#include "../learning/CaclaLearningI.h"

using namespace std;

class CaclaDriver : public RLDriver
{
public:
	
	// Constructor
	CaclaDriver();

	// SimpleDriver implements a simple and heuristic controller for driving
	//virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	//virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	//virtual void init(float *angles);

private:

	////////Functions added by Daniel:
	virtual void initInterface(bool load_network);
	//void doLearning(CarState &cs);
	//CarControl rlControl(CarState &cs);

	///////Datamembers added by Daniel:
	//vector<double>* mp_features;
	//CaclaLearningI* mp_Qinterface;
	//double* mp_action_set;
	//Writer* mp_log;
	//Writer* mp_reward_writer;

};

#endif /*CaclaDriver_H_*/