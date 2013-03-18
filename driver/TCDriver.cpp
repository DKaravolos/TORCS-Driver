#include "TCDriver.h"

TCDriver::TCDriver(): RLDriver()
{
	m_log_dir = "log_files/";
	mp_log = new Writer("log_files/TCDriver_log.txt");
	mp_reward_writer = new Writer("log_files/TCDriver_rewards.txt");
}

TCDriver::TCDriver(const string& log_dir, const int& steps, const int& runs, const bool& save_data):
RLDriver(steps, runs, save_data)
{
	m_log_dir = log_dir;
	mp_log = new Writer(log_dir + "TCDriver_log.txt");
	mp_reward_writer = new Writer(log_dir + "TCDriver_rewards.txt");
	mp_lap_writer = new Writer(log_dir + "TCDriver_laps.txt");
	mp_eoe_writer = new Writer(log_dir + "TCDriver_endofep.txt");
}

void TCDriver::onShutdown()
{
	cout << "Shutting down\n";
	RLDriver::onShutdown();
	delete mp_log;
	delete mp_reward_writer;
	delete mp_lap_writer;
	delete mp_eoe_writer;
	delete mp_RLinterface;
}

void TCDriver::initInterface(const bool& load_network, const bool& m_automatic_experiment)
{
	mp_RLinterface = new TCLearningInterface(m_log_dir);

	//Sometimes you already know that you don't want to load a network
	if(load_network && !m_automatic_experiment)
		askLoadNetwork();
	else{
		m_network_id = 0;
		m_step_id = 0;
		mp_RLinterface->init(true);
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
		cout << "Which directory? (ending with /)\n";
		string l_log_dir;
		cin >> l_log_dir;

		//Ask user which network he wants to load
		cout << "Which ID (x1.000)?\n";
		cin >> m_network_id;
		cout << "Which step (x1.000)?\n";
		cin >> m_step_id;
		stringstream file_name;
		file_name << l_log_dir << "TC_QTable_id_"<< m_network_id << "000_step_" << m_step_id << "000.txt";
		string base_file = file_name.str();
		is.open(base_file.c_str());
		if(is.is_open()) {
			is.close();
			cout << "\nLoading QTable from file.\n";
			const char* char_base_file = base_file.c_str();
			TCLearningInterface* l_TCLI = static_cast<TCLearningInterface*>(mp_RLinterface); 
			l_TCLI->init(false, char_base_file);
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
