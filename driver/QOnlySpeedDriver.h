#ifndef QOnlySpeedDriver_H_
#define QOnlySpeedDriver_H_

#include "RLDriver.h"
#include "../learning/QOSLearningInterface.h"

using namespace std;

class QOnlySpeedDriver : public RLDriver
{
public:
	
	// Constructor
	QOnlySpeedDriver();

	// SimpleDriver implements a simple and heuristic controller for driving
	//virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	//virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	//virtual void init(float *angles);

protected:
	
	////////Functions added by Daniel:
	virtual void initInterface(bool load_network);
	void askLoadNetwork();
	//void doLearning(CarState &cs);
	virtual CarControl rlControl(CarState &cs);

	// Solves the steering subproblems
	float getSteer(CarState &cs);

	///////Datamembers added by Daniel:
	//vector<double>* mp_features;
	//QOSLearningInterface* mp_Qinterface;
	//double* mp_action_set;
	//Writer* mp_log;
	//Writer* mp_reward_writer;
};

#endif /*QOnlySpeedDriver_H_*/