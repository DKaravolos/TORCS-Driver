#ifndef RLDriver_H_
#define RLDriver_H_

#ifdef WIN32
	#include <conio.h>
#endif
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "CarState.h"
#include "CarControl.h"
#include "WrapperBaseDriver.h"
//
#include <vector>

//Functions/classes by Daniel:
#ifdef WIN32
#include "..\learning\RLInterface.h"
#include "..\utilities\createFeatureVector.h"
#include "..\utilities\printFeatureVector.h"
#include "..\utilities\Writer.h"
#else
#include "../learning/RLInterface.h"
#include "../utilities/createFeatureVector.h"
#include "../utilities/printFeatureVector.h"
#include "../utilities/Writer.h"
#endif


#define PI 3.14159265

using namespace std;

class RLDriver : public WrapperBaseDriver
{
public:
	
	// Constructor
	RLDriver();
	RLDriver(const int& nr_steps, const int& nr_runs, const bool& save_data);
	//~RLDriver();

	// SimpleDriver implements a simple and heuristic controller for driving
	virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	virtual void init(float *angles);

	//Functions for automatic experiments
	void changeLogWriterTo(std::string& new_file);
	void changeRewardWriterTo(std::string& new_file);
	int getLearningStep();
	inline RLInterface* getInterface(){return mp_RLinterface;}

protected:
	
	/* Gear Changing Constants*/
	
	// RPM values to change gear 
	static const int gearUp[6];
	static const int gearDown[6];
		
	/* Stuck constants*/
	
	// How many time steps the controller wait before recovering from a stuck position
	static const int stuckTime;
	// When car angle w.r.t. track axis is grather tan stuckAngle, the car is probably stuck
	static const float stuckAngle;
	
	/* Steering constants*/
	
	// Angle associated to a full steer command
	static const float steerLock;	
	// Min speed to reduce steering command 
	static const float steerSensitivityOffset;
	// Coefficient to reduce steering command at high speed (to avoid loosing the control)
	static const float wheelSensitivityCoeff;
	
	/* Accel and Brake Constants*/
	
	// max speed allowed
	static const float maxSpeed;
	// Min distance from track border to drive at  max speed
	static const float maxSpeedDist;
	// pre-computed sin5
	static const float sin5;
	// pre-computed cos5
	static const float cos5;
	
	/* ABS Filter Constants */
	
	// Radius of the 4 wheels of the car
	static const float wheelRadius[4];
	// min slip to prevent ABS
	static const float absSlip;						
	// range to normalize the ABS effect on the brake
	static const float absRange;
	// min speed to activate ABS
	static const float absMinSpeed;

	/* Clutch constants */
	static const float clutchMax;
	static const float clutchDelta;
	static const float clutchRange;
	static const float clutchDeltaTime;
	static const float clutchDeltaRaced;
	static const float clutchDec;
	static const float clutchMaxModifier;
	static const float clutchMaxTime;

	// counter of stuck steps
	int stuck;
	
	// current clutch
	float clutch;

	// Solves the gear changing subproblems
	int getGear(CarState &cs);

	// Solves the steering subproblems
	virtual float getSteer(CarState &cs);
	
	// Solves the gear changing subproblems
	float getAccel(CarState &cs);
	
	// Apply an ABS filter to brake command
	float filterABS(CarState &cs,float brake);

	// Solves the clucthing subproblems
	void clutching(CarState &cs, float &clutch);

	////////Functions added by Daniel:
	void setPrefs();
	virtual void initInterface(const bool& load_network, const bool& automatic_experiment)=0;
	bool stuckCheck(CarState& cs);
	double computeReward(CarState &state, double* action, CarState &next_state);
	void doLearning(CarState &cs);
	void doUpdate(CarState &cs);
	virtual CarControl carStuckControl(CarState &cs);
	CarControl simpleBotControl(CarState &cs);
	virtual CarControl rlControl(CarState &cs);
	virtual void endOfRunCheck(CarState &cs, CarControl &cc);
	char getKeyboardInput();
	char getArrowInput();

	///////Datamembers added by Daniel:
	vector<double>* mp_features;
	RLInterface* mp_RLinterface;
	double* mp_action_set;
	Writer* mp_log;
	Writer* mp_reward_writer;

	//Elements for computing rewards
	CarState* gp_prev_state;
	
	//time counters
	int g_count;
	int g_learn_step_count;
	int g_stuck_step_count;
	int g_reupdate_step_count;
	int g_experiment_count;

	//flags for learning
	bool g_learning_done;
	bool g_first_time;

	//learning parameters
	int g_stuck_penalty;
	int g_steps_per_action;
	int g_learn_steps_per_tick;
	int g_reupdate_steps_per_tick;
	int g_reupdates_left;

	//debug parameters
	int g_print_mod;
	//int debug_stuck_count;
	int debug_rlcontrol_count;

	//user preferences
	bool m_save_nn;
	int m_network_id;
	int m_step_id;
	int m_exp_count;
	int m_round_size;
	bool m_automatic_experiment;
	string m_log_dir;
};

#endif /*RLDriver_H_*/