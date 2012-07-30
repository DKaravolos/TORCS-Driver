#include "QOnlySpeedDriver.h"
//#include <Windows.h>

QOnlySpeedDriver::QOnlySpeedDriver()
{
	debug_stuck_count = 0;
	debug_rlcontrol_count = 0;

	stuck=0;clutch=0.0;
	mp_features = NULL;
	mp_RLinterface = NULL;
	gp_prev_state = NULL;

	g_steps_per_action = 10;
	g_print_mod = g_steps_per_action;
	g_learn_steps_per_tick = 1;
	g_reupdate_steps_per_tick = 0;

	g_stuck_penalty = 0;
	
	g_experiment_count = 0;

	mp_log = new Writer("log_files/QOnlySpeedDriver_log.txt");
	mp_reward_writer = new Writer("log_files/QOnlySpeedDriver_rewards_0.txt");
}

void QOnlySpeedDriver::initInterface(bool load_network)
{
	mp_RLinterface = new QOSLearningInterface();
	ifstream is;
	is.open("log_files/QLearning_QNN_step_9000_action_1");
	if(load_network && is.is_open()) {
		is.close();
		cout << "Loading NN from file.";
		mp_RLinterface->init("log_files/QLearning_QNN_step_9000");
	}
	else
		mp_RLinterface->init();
}

CarControl QOnlySpeedDriver::rlControl(CarState &cs)
{
	//cout << "Time: " << cs.getCurLapTime() << endl;
	debug_rlcontrol_count++;
	// compute gear 
    int gear = getGear(cs);
	float steer = getSteer(cs);
	//note that steer is now computed by a simple function in parent, it is not given by any RL function

    // set accel and brake from the joint accel/brake command 
    float accel,brake;
    if (mp_action_set[1]>0)
    {
        accel = mp_action_set[1];
        brake = 0.0f;
    }
    else
    {
        accel = 0.0f;
        // apply ABS to brake
        brake = filterABS(cs,-mp_action_set[1]);
    }

    // Calculate clutching
    clutching(cs,clutch);

    // build a CarControl variable and return it
    CarControl cc(accel,brake,gear,steer,clutch);
	endOfRunCheck(cs, cc);

    return cc;
}

void QOnlySpeedDriver::onRestart()
{
	//delete mp_features;
	mp_features = NULL;
	//delete mp_RLinterface; // We are not reinitializing the interface between runs.
	//This may have negative side-effects, I have not completely thought this through.
    cout << "Restarting the race!" << endl;
	g_learn_step_count = -1;
	delete mp_reward_writer;

	stringstream newfile;
	newfile << "log_files/QOnlySpeedDriver_rewards_" << g_experiment_count << ".txt";
	mp_reward_writer = new Writer(newfile.str());

	//try{
	//	initInterface(true);
	//} catch(exception& e) {
	//	cout << e.what() << endl;
	//}
}