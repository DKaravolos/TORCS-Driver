#include "BASDriver.h"

BASDriver::BASDriver(): RLDriver()
{
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
	RLDriver::onRestart();
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