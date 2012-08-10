#include "CaclaDriver.h"
#include <windows.h>

CaclaDriver::CaclaDriver(): RLDriver()
{
	mp_log = new Writer("log_files/Cacla_driver_log.txt");
	mp_reward_writer = new Writer("log_files/Cacla_driver_rewards_0.txt");
}

void CaclaDriver::initInterface(bool load_network)
{
	mp_RLinterface = new CaclaLearningI();
	ifstream is;
	ifstream is2;
	is.open("log_files/Cacla_ANN_ep_544_step_9000.txt");
	is2.open("log_files/Cacla_VNN_ep_544_step_9000.txt");
	if(load_network && is.is_open() && is2.is_open()) {
		is.close();
		is2.close();
		cout << "Loading NN from file.";
		//We need a special init for this class, so we cast it to its derived class (unsafe!)
		CaclaLearningI* l_CaclaLI = static_cast<CaclaLearningI*>(mp_RLinterface); 
		//However, we know it is a CaclaLearningI, because it is created a few lines above
		l_CaclaLI->init("log_files/Cacla_ANN_ep_544_step_9000.txt", "log_files/Cacla_VNN_ep_544_step_9000.txt");
	}
	else
		mp_RLinterface->init();
}

void CaclaDriver::onRestart()
{
	//delete mp_features;
	/*delete mp_Qinterface;*/
    cout << "Restarting the race!" << endl;
	g_learn_step_count = -1;
	delete mp_reward_writer;

	stringstream newfile;
	newfile << "log_files/Cacla_driver_rewards_" << g_experiment_count << ".txt";
	mp_reward_writer = new Writer(newfile.str());

	//try{
	//	initInterface(true);
	//} catch(exception& e) {
	//	cerr << e.what() << endl;
	//}
}