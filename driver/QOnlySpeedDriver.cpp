#include "QOnlySpeedDriver.h"
#include <sstream>
//#include <Windows.h>

QOnlySpeedDriver::QOnlySpeedDriver(): RLDriver()
{
	mp_log = new Writer("log_files/QOnlySpeedDriver_log.txt");
	mp_reward_writer = new Writer("log_files/QOnlySpeedDriver_rewards_0.txt");
}

void QOnlySpeedDriver::initInterface(bool load_network)
{
	mp_RLinterface = new QOSLearningInterface();
	ifstream is;
	cout << "initInterface\n";
	m_network_id = 0;
	m_step_id = 0;
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
		file_name << "log_files/QOS_QNN_id_"<< m_network_id << "000_step_" << m_step_id;
		string base_file = file_name.str();
		file_name << "_action_1";
		is.open(file_name.str());
		if(load_network && is.is_open()) {
			is.close();
			cout << "\nLoading NN from file.\n";
			const char* char_base_file = base_file.c_str();
			mp_RLinterface->init(char_base_file);
		} else {
			cout << "Could not load that file. Creating new network.\n";
			mp_RLinterface->init();
		}
	} else {
		cout << "Not loading NN.\n";
		mp_RLinterface->init();
	}

	//How many runs does the user want?
	cout << "How many runs of 10.000? \n";
	cin >> m_exp_count;

}

float QOnlySpeedDriver::getSteer(CarState &cs)
{
    float targetAngle=float( cs.getAngle());
    //// at high speed reduce the steering command to avoid loosing the control
    ////if (cs.getSpeedX() > steerSensitivityOffset)
    ////    return targetAngle/(steerLock*(cs.getSpeedX()-steerSensitivityOffset)*0.25);
    ////else
        return (targetAngle)/steerLock; //steerLock is important, it prevents a lot of crashes.

	//// steering angle is compute by correcting the actual car angle w.r.t. to track 
	//// axis [cs.getAngle()] and to adjust car position w.r.t to middle of track [cs.getTrackPos()*0.5]
 //   float targetAngle=float(cs.getAngle()-cs.getTrackPos()*0.5);
 //   // at high speed reduce the steering command to avoid loosing the control
 //   if (cs.getSpeedX() > steerSensitivityOffset)
 //       return targetAngle/(steerLock*(cs.getSpeedX()-steerSensitivityOffset)*wheelSensitivityCoeff);
 //   else
 //       return (targetAngle)/steerLock;
}

CarControl QOnlySpeedDriver::rlControl(CarState &cs)
{
	debug_rlcontrol_count++;
	//cout << "Time: " << cs.getCurLapTime() << endl;
	// compute gear 
    int gear = getGear(cs);
	float steer = getSteer(cs);
	//if(cs.getSpeedX() < 2) //if speed is less than 2km/h: do not steer this makes no sense
	//	steer = 0;
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
	mp_RLinterface->setFirstTime(true); //this function counters the negative side-effects :)
    cout << "Restarting the race!" << endl;
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