#include "QDriver.h"
#ifdef WIN32
    #include <Windows.h>
#endif
QDriver::QDriver(): RLDriver()
{
	mp_log = new Writer("log_files/Q_driver_log.txt");
	mp_reward_writer = new Writer("log_files/Q_driver_rewards_0.txt");
}

/*
void QDriver::init(float *angles)
{
	g_learn_step_count = -1;
	g_reupdate_step_count = 0; //unnecessary, but good practice
	g_count = -1;
	g_learning_done = false;
	g_first_time = true;

	//Set Daniels datamembers
	mp_features = new vector<double>;
	if (mp_RLinterface == NULL) {
		cout << "Creating LearningInterface...\n";
		try
		{
			initInterface(true);
			cout << "Done.\n";
		} catch (exception& e)
		{
			cerr << e.what() << endl;
			#ifdef WIN32
				char end;
				cin >> end;
			#endif
			exit(-3);
		}
	} else {
		cout << "\nAlready created a LearningInterface. Skipping constructor and init.\n";
	}

	// set angles as {-90,-75,-60,-45,-30,20,15,10,5,0,5,10,15,20,30,45,60,75,90}

	for (int i=0; i<5; i++)
	{
		angles[i]=-90+i*15;
		angles[18-i]=90-i*15;
	}

	for (int i=5; i<9; i++)
	{
			angles[i]=-20+(i-5)*5;
			angles[18-i]=20-(i-5)*5;
	}
	angles[9]=0;
}
*/
void QDriver::initInterface(bool load_network)
{
	mp_RLinterface = new LearningInterface();

	//Sometimes you already know that you don't want to load a network
	if(load_network)
		askLoadNetwork();
	else{
		m_network_id = 0;
		m_step_id = 0;
		mp_RLinterface->init();
	}
}


void QDriver::askLoadNetwork()
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
		stringstream file_name;
		file_name << "log_files/QLearning_QNN_id_"<< m_network_id << "000_step_" << m_step_id;
		string base_file = file_name.str();
		file_name << "_action_1";
		is.open(file_name.str());
		if(is.is_open()) {
			is.close();
			cout << "\nLoading NN from file.\n";
			const char* char_base_file = base_file.c_str();
			mp_RLinterface->init(char_base_file);
		} else {
			cout << "Could not load that file. Creating new network.\n";
			m_network_id = 0;
			m_step_id = 0;
			mp_RLinterface->init();
		}
	} else {
		cout << "Not loading NN.\n";
		m_network_id = 0;
		m_step_id = 0;
		mp_RLinterface->init();
	}
}

void QDriver::onRestart()
{
	//delete mp_features;
	mp_features = NULL;
	//delete mp_RLinterface; // We are not reinitializing the interface between runs. This mat have negative side-effects
	mp_RLinterface->setFirstTime(true); // one of the side-effects is having to manually set first time
    cout << "Restarting the race!" << endl;
	//g_learn_step_count = -1; // I'm trying to keep counting between restarts
	
	delete mp_reward_writer;

	stringstream newfile;
	newfile << "log_files/Reciting_driver_rewards_" << g_experiment_count << ".txt";
	mp_reward_writer = new Writer(newfile.str());

	//try{
	//	initInterface(true);
	//} catch(exception& e) {
	//	cout << e.what() << endl;
	//}
}