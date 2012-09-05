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
	bool clean_init = true;
	//Sometimes you already know that you don't want to load a network
	if(load_network)
		clean_init = askLoadNetwork();
	
	if(clean_init)
	{
		m_network_id = 0;
		m_step_id = 0;
		mp_RLinterface->init();
	}
}

//Returns whether an init from zero is necessary (or: whether is has (not) been able to load a network)
bool CaclaDriver::askLoadNetwork()
{
	ifstream is;
	//Does the user want to load a network?
	cout << "Want to load a NN? (y/n)\n";
	char answer;
	cin >> answer;
	if(answer == 'y')
	{
		//Ask user which network he wants to load
		cout << "Which ID (x1.000)?\n";
		cin >> m_network_id;
		cout << "Which step ?\n";
		cin >> m_step_id;
		stringstream ANN_name;
		stringstream VNN_name;
		ANN_name << "log_files/Cacla_ANN_id_"<< m_network_id << "000_step_" << m_step_id;
		VNN_name << "log_files/Cacla_VNN_id_"<< m_network_id << "000_step_" << m_step_id;

		is.open(ANN_name.str());
		if(is.is_open()) {
			is.close();
			cout << "\nLoading NN from file.\n";
			string ANN = ANN_name.str();
			string VNN = VNN_name.str();
			const char* char_ANN_file = ANN.c_str();
			const char* char_VNN_file = VNN.c_str();
			//We need a special init for this class, so we cast it to its derived class (unsafe!)
			CaclaLearningI* l_CaclaLI = static_cast<CaclaLearningI*>(mp_RLinterface); 
			//However, we know it is a CaclaLearningI, because it is created a few lines above this function call
			//Please check the type of mp_RLinterface when called outside initInterface
			l_CaclaLI->init(char_ANN_file, char_VNN_file);
			return false;
		} else {
			cout << "Could not load that file. Creating new network.\n";
			return true;
		}
	} else {
		cout << "Not loading NN.\n";
		return true;
	}
}

void CaclaDriver::onRestart()
{
	//delete mp_features;
	mp_features = NULL;
	//delete mp_RLinterface; // We are not reinitializing the interface between runs. This mat have negative side-effects
	mp_RLinterface->setFirstTime(true); // one of the side-effects is having to manually set first time
    cout << "Restarting the race!" << endl;
	//g_learn_step_count = -1; // I'm trying to keep counting between restarts

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