#include "BASDriver.h"

BASDriver::BASDriver()
{
	debug_stuck_count = 0;
	debug_rlcontrol_count = 0;

	stuck=0;clutch=0.0;
	mp_features = NULL;
	mp_RLinterface = NULL;
	gp_prev_state = NULL;

	g_steps_per_action = 10;
	g_print_mod = g_steps_per_action;
	g_learn_steps_per_tick = 2;
	g_reupdate_steps_per_tick = 2;

	g_stuck_penalty = 0;
	
	g_experiment_count = 0;

	mp_log = new Writer("log_files/BAS_driver_log.txt");
	mp_reward_writer = new Writer("log_files/BAS_driver_rewards_0.txt");
}

void BASDriver::initInterface(bool load_network)
{
	mp_RLinterface = new BASLearningInterface();
	ifstream is;
	is.open("log_files/BASDriver_QNN_step_9000_actionDim_0_left.txt"); // CHECK NOG EVEN DEZE BESTANDSNAAM!
	if(load_network && is.is_open()) {
		is.close();
		cout << "Loading NN from file.\n\n";
		mp_RLinterface->init("log_files/BASDriver_QNN_step_9000");
	}
	else
		mp_RLinterface->init();
}

void BASDriver::onRestart()
{
	//delete mp_features; //should probably not be deleted when restarting.
	//delete mp_Qinterface; // We are not reinitializing the interface between runs.
	//This may have negative side-effects, I have not completely thought this through.
    cout << "Restarting the race!" << endl;
	g_learn_step_count = -1;
	delete mp_reward_writer;

	stringstream newfile;
	newfile << "log_files/BAS_driver_rewards_" << g_experiment_count << ".txt";
	mp_reward_writer = new Writer(newfile.str());

	//try{
	//	initInterface(true);
	//} catch(exception& e) {
	//	cout << e.what() << endl;
	//}
}