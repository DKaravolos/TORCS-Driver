#include "RecitingDriver.h"
#ifdef WIN32
    #include <Windows.h>
#endif
RecitingDriver::RecitingDriver(): RLDriver()
{
	mp_log = new Writer("log_files/Reciting_driver_log.txt");
	mp_reward_writer = new Writer("log_files/Reciting_driver_rewards_0.txt");
}

/*
void RecitingDriver::init(float *angles)
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
void RecitingDriver::initInterface(bool load_network)
{
	mp_RLinterface = new LearningInterface();
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

//void RecitingDriver::onShutdown()
//{
//	cout << "Bye bye!" << endl;
//	delete mp_features;
//	mp_features = NULL;
//	delete mp_log;
//	delete mp_reward_writer;
//	delete mp_RLinterface; 
//	delete gp_prev_state;
//}

void RecitingDriver::onRestart()
{
	//delete mp_features;
	mp_features = NULL;
	//delete mp_RLinterface; // We are not reinitializing the interface between runs.
	//This may have negative side-effects, I have not completely thought this through.
    cout << "Restarting the race!" << endl;
	g_learn_step_count = -1;
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