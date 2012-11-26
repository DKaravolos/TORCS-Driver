#include "CaclaDriver.h"

CaclaDriver::CaclaDriver(): RLDriver()
{
	m_log_dir = "log_files/";
	mp_log = new Writer("log_files/Cacla_driver_log.txt");
	mp_reward_writer = new Writer("log_files/Cacla_driver_rewards_0.txt");
}

CaclaDriver::CaclaDriver(const string& log_dir, const int& steps, const int& runs, const bool& save_data):
RLDriver(steps, runs, save_data)
{
	m_log_dir = log_dir;
	mp_log = new Writer(log_dir + "CaclaDriver_log.txt");
	mp_reward_writer = new Writer(log_dir + "/CaclaDriver_rewards.txt");
}

void CaclaDriver::onShutdown()
{
	cout << "Shutting down\n";
	RLDriver::onShutdown();
	delete mp_log;
	delete mp_reward_writer;
	delete mp_RLinterface;
}

void CaclaDriver::initInterface(const bool& load_network, const bool& m_automatic_experiment)
{
	mp_RLinterface = new CaclaLearningI(m_log_dir);
	bool clean_init = true;
	//Sometimes you already know that you don't want to load a network
	if(load_network && !m_automatic_experiment)
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
cout << "Which directory? (ending with /)\n";
		string l_log_dir;
		cin >> l_log_dir;

		//Ask user which network he wants to load
		cout << "Which ID (x1.000)?\n";
		cin >> m_network_id;
		cout << "Which step (x1.000)?\n";
		cin >> m_step_id;
		m_step_id *= 1000;

		stringstream ANN_name;
		stringstream VNN_name;
		ANN_name << l_log_dir << "Cacla_ANN_id_"<< m_network_id << "000_step_" << m_step_id;
		VNN_name << l_log_dir << "Cacla_VNN_id_"<< m_network_id << "000_step_" << m_step_id;

		is.open(ANN_name.str());
		if(is.is_open()) {
			is.close();
			cout << "\nLoading NN from file.\n";
			//We need a special init for this class, so we cast it to its derived class (unsafe!)
			CaclaLearningI* l_CaclaLI = static_cast<CaclaLearningI*>(mp_RLinterface); 
			//However, we know it is a CaclaLearningI, because it is created a few lines above this function call
			//Please check the type of mp_RLinterface when called outside initInterface
			l_CaclaLI->init(false, ANN_name.str().c_str(), VNN_name.str().c_str());
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

//void CaclaDriver::onRestart()
//{
//	RLDriver::onRestart();
//	delete mp_reward_writer;
//
//	stringstream newfile;
//	newfile << "log_files/Cacla_driver_rewards_" << g_experiment_count << ".txt";
//	mp_reward_writer = new Writer(newfile.str());
//}