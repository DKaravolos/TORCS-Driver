#include "TCDriver.h"
#ifdef WIN32
    #include <Windows.h>
#endif
TCDriver::TCDriver(): RLDriver()
{
	mp_log = new Writer("log_files/TC_driver_log.txt");
	mp_reward_writer = new Writer("log_files/TC_driver_rewards_0.txt");
}

void TCDriver::onShutdown()
{
	cout << "Shutting down\n";
	RLDriver::onShutdown();
	delete mp_log;
	delete mp_reward_writer;
	delete mp_RLinterface;
}

void TCDriver::initInterface(bool load_network)
{
	mp_RLinterface = new TCLearningInterface();

	//Sometimes you already know that you don't want to load a network
	if(load_network)
		askLoadNetwork();
	else{
		m_network_id = 0;
		m_step_id = 0;
		mp_RLinterface->init();
	}
}

void TCDriver::askLoadNetwork()
{
	ifstream is;
	//Does the user want to load a network?
	cout << "Want to load a QTable? (y/n)\n";
	char answer;
	cin >> answer;
	if(answer == 'y')
	{
		//Ask user which network he wants to load
		cout << "Which ID (x1.000)?\n";
		cin >> m_network_id;
		cout << "Which step ?\n";
		cin >> m_step_id;
		stringstream file_name;
		file_name << "log_files/TC_QTable_id_"<< m_network_id << "000_step_" << m_step_id;
		string base_file = file_name.str();
		is.open(base_file);
		if(is.is_open()) {
			is.close();
			cout << "\nLoading QTable from file.\n";
			const char* char_base_file = base_file.c_str();
			TCLearningInterface* l_TCLI = static_cast<TCLearningInterface*>(mp_RLinterface); 
			l_TCLI->init(char_base_file);
		} else {
			cout << "Could not load '"<< base_file << "'. Creating new QTable.\n";
			m_network_id = 0;
			m_step_id = 0;
			mp_RLinterface->init();
		}
	} else {
		cout << "Creating new QTable.\n";
		m_network_id = 0;
		m_step_id = 0;
		mp_RLinterface->init();
	}
}

void TCDriver::onRestart()
{
	RLDriver::onRestart();
	
	delete mp_reward_writer;

	stringstream newfile;
	newfile << "log_files/TCDriver_rewards_" << g_experiment_count << ".txt";
	mp_reward_writer = new Writer(newfile.str());
}