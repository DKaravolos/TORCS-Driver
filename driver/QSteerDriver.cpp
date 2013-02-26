#include "QSteerDriver.h"
#include <sstream>
//#include <Windows.h>

QSteerDriver::QSteerDriver(): RLDriver()
{
	mp_log = new Writer("log_files/QSteerDriver_log.txt");
	mp_reward_writer = new Writer("log_files/QSteerDriver_rewards_0.txt");
	
	cout << "Do you want to control the car's acceleration yourself? (y/n)\n";
	char answer;
	cin >> answer;
	if( answer == 'y')
		m_user_control = true;
	else
		m_user_control = false;
}

void QSteerDriver::initInterface(bool load_network)
{
	cout << "init interface\n";
	mp_RLinterface = new QOSLearningInterface();
	
	//Sometimes you already know that you don't want to load a network
	//However, m_network_id and m_step_id will not be set if you skip askLoadNetwork()
	if(load_network)
		askLoadNetwork();
	else
	{
		m_network_id = 0;
		m_step_id = 0;
		mp_RLinterface->init();
	}
}

void QSteerDriver::askLoadNetwork()
{
	ifstream is;
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
		file_name << "log_files/QSteer_QNN_id_"<< m_network_id << "000_step_" << m_step_id;
		string base_file = file_name.str();
		file_name << "_action_1";
		is.open(file_name.str().c_str());
		if(is.is_open()) {
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
}

float QSteerDriver::getAccel(CarState &cs)
{
    // checks if car is out of track
    if (cs.getTrackPos() < 1 && cs.getTrackPos() > -1)
    {
        // reading of sensor at +5 degree w.r.t. car axis
        float rxSensor=cs.getTrack(10);
        // reading of sensor parallel to car axis
        float cSensor=cs.getTrack(9);
        // reading of sensor at -5 degree w.r.t. car axis
        float sxSensor=cs.getTrack(8);

        float targetSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (cSensor>maxSpeedDist || (cSensor>=rxSensor && cSensor >= sxSensor))
            targetSpeed = maxSpeed;
        else
        {
            // approaching a turn on right
            if(rxSensor>sxSensor)
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = rxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }
            // approaching a turn on left
            else
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = sxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }

        }

        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        return 2/(1+exp(cs.getSpeedX() - targetSpeed)) - 1;
    }
    else
        return 0.3f; // when out of track returns a moderate acceleration command

}

CarControl QSteerDriver::rlControl(CarState &cs)
{
	debug_rlcontrol_count++;
	// compute gear 
    int gear = getGear(cs);

	// compute steering
	float steer = float(mp_action_set[0]);

    // compute accel/brake command
	float accel_and_brake;
	if(m_user_control)
	{
		accel_and_brake = getAccelFromKeyboard();
	}
	else
		accel_and_brake = getAccel(cs);
        
	// normalize steering
	if (steer < -1)
		steer = -1;
	if (steer > 1)
		steer = 1;

	// set accel and brake from the joint accel/brake command 
	float accel,brake;
	if (accel_and_brake>0)
	{
		accel = accel_and_brake;
		brake = 0.0f;
	}
	else
	{
		accel = 0.0f;
		// apply ABS to brake
		brake = filterABS(cs,-accel_and_brake);
	}
	// Calculate clutching
	clutching(cs,clutch);

	// build a CarControl variable and return it
	CarControl cc(accel,brake,gear,steer,clutch);
	endOfRunCheck(cs, cc);
	return cc;
}

void QSteerDriver::onRestart()
{
	RLDriver::onRestart();
	delete mp_reward_writer;

	stringstream newfile;
	newfile << "log_files/QSteerDriver_rewards_" << g_experiment_count << ".txt";
	mp_reward_writer = new Writer(newfile.str());

	//try{
	//	initInterface(true);
	//} catch(exception& e) {
	//	cout << e.what() << endl;
	//}
}

float QSteerDriver::getAccelFromKeyboard()
{
	char input = getArrowInput();
	//cout << "Input = : " << input << endl;
	if(input == 'H' || input == 'h'){ // capital is fully pressed
		cout << "Up.\n";
		return 1.0f;
	} else if(input == 'P' || input == 'p'){
		cout << "Down.\n";
		return -1.0f;
	} else
		return 0.0f;
}

CarControl QSteerDriver::carStuckControl(CarState &cs)
{
	//if(m_user_control)
	//	return keyboardStuckControl(cs);
	//else {
		//cout << "RLDriver::stuckControl!\n";
		return RLDriver::carStuckControl(cs);
	//}
}
