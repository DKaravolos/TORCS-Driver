#ifndef BASDriver_H_
#define BASDriver_H_

#include "RLDriver.h"
#include "../learning/BASLearningInterface.h"

using namespace std;

class BASDriver : public RLDriver
{
public:
	
	// Constructor
	BASDriver();

	// SimpleDriver implements a simple and heuristic controller for driving
	//virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	//virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	//virtual void init(float *angles);

private:
	/* Functions added by Daniel: */
	virtual void initInterface(bool load_network);
	//CarControl rlControl(CarState &cs);

	/* Datamembers added by Daniel: */
	//BASLearningInterface* mp_Qinterface;

};

#endif /*BASDriver_H_*/