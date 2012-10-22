#ifndef QDriver_H_
#define QDriver_H_

#include <iostream>
#include <fstream>
#include <sstream>
//
#include <vector>
//Functions/classes by Daniel:
#include "RLDriver.h"
#include "..\learning\LearningInterface.h"
#include "..\utilities\Writer.h"

using namespace std;

class QDriver : public RLDriver
{
public:
	
	// Constructor
	QDriver();

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
	virtual void initInterface(bool load_network);
	void askLoadNetwork();
	//void doLearning(CarState &cs);
	//CarControl rlControl(CarState &cs);
	//virtual void endOfRunCheck(CarState &cs, CarControl &cc);

	///////Datamembers added by Daniel:
	//LearningInterface* mp_Qinterface;

	
};

#endif /*QDriver_H_*/
