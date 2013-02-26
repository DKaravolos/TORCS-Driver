#ifndef QSteerDriver_H_
#define QSteerDriver_H_

#include "RLDriver.h"
#include "../learning/QOSLearningInterface.h"

using namespace std;

class QSteerDriver : public RLDriver
{
public:
	
	// Constructor
	QSteerDriver();

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
	CarControl rlControl(CarState &cs);
	CarControl carStuckControl(CarState &cs);

	// Solves the acceleration subproblems
	float getAccel(CarState &cs);
	float getAccelFromKeyboard();

	//user-controlled acceleration
	bool m_user_control;
	
	
};

#endif /*QSteerDriver_H_*/