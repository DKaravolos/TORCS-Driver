#include "RecitingDriver.h"

int debug_stuck_count = 0;
int debug_rlcontrol_count = 0;

RecitingDriver::RecitingDriver()
{
	stuck=0;clutch=0.0;
	mp_features = NULL;
	mp_Qinterface = NULL;
	m_last_dist = 0;
	m_last_dist_from_start = 0;
	m_last_damage = 0;

	g_steps_per_action = 10;
	g_print_mod = 100*g_steps_per_action;
	g_learn_steps_per_tick = 2;
	g_reupdate_steps_per_tick = 2;

	g_stuck_penalty = 0;
	
	g_experiment_count = 0;

	mp_log = new Writer("log_files/Reciting_driver_log.txt");
	mp_reward_writer = new Writer("log_files/Reciting_driver_rewards_0.txt");
}

/* Gear Changing Constants*/
const int RecitingDriver::gearUp[6]=
    {
        8000,9500,9500,9500,9500,0
    };
const int RecitingDriver::gearDown[6]=
    {
        0,4000,6300,7000,7300,7300
    };

/*
const int RecitingDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int RecitingDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };
//*/

/* Stuck constants*/
const int RecitingDriver::stuckTime = 25;
const float RecitingDriver::stuckAngle = 0.785398163f; //.523598775f; //PI/6

/* Accel and Brake Constants*/
const float RecitingDriver::maxSpeedDist=70;
const float RecitingDriver::maxSpeed=150;
const float RecitingDriver::sin5 = 0.08716f;
const float RecitingDriver::cos5 = 0.99619f;

/* Steering constants*/
const float RecitingDriver::steerLock=0.785398f;
const float RecitingDriver::steerSensitivityOffset=80.0f;
const float RecitingDriver::wheelSensitivityCoeff=1;

/* ABS Filter Constants */
const float RecitingDriver::wheelRadius[4]={0.3179f,0.3179f,0.3276f,0.3276f};
const float RecitingDriver::absSlip=2.0f;
const float RecitingDriver::absRange=3.0f;
const float RecitingDriver::absMinSpeed=3.0f;

/* Clutch constants */
const float RecitingDriver::clutchMax=0.5f;
const float RecitingDriver::clutchDelta=0.05f;
const float RecitingDriver::clutchRange=0.82f;
const float RecitingDriver::clutchDeltaTime=0.02f;
const float RecitingDriver::clutchDeltaRaced=10;
const float RecitingDriver::clutchDec=0.01f;
const float RecitingDriver::clutchMaxModifier=1.3f;
const float RecitingDriver::clutchMaxTime=1.5f;

int RecitingDriver::getGear(CarState &cs)
{

    int gear = cs.getGear();
    int rpm  = cs.getRpm();

    // if gear is 0 (N) or -1 (R) just return 1 
    if (gear<1) {
		//cout << "Setting gear to one";
        return 1;
	}
    // check if the RPM value of car is greater than the one suggested 
    // to shift up the gear from the current one     
    if (gear <6 && rpm >= gearUp[gear-1])
        return gear + 1;
    else
    	// check if the RPM value of car is lower than the one suggested 
    	// to shift down the gear from the current one
        if (gear > 1 && rpm <= gearDown[gear-1])
            return gear - 1;
        else // otherwhise keep current gear
            return gear;
}
float RecitingDriver::getSteer(CarState &cs)
{
	// steering angle is compute by correcting the actual car angle w.r.t. to track 
	// axis [cs.getAngle()] and to adjust car position w.r.t to middle of track [cs.getTrackPos()*0.5]
    float targetAngle=float(cs.getAngle()-cs.getTrackPos()*0.5);
    // at high speed reduce the steering command to avoid loosing the control
    if (cs.getSpeedX() > steerSensitivityOffset)
        return targetAngle/(steerLock*(cs.getSpeedX()-steerSensitivityOffset)*wheelSensitivityCoeff);
    else
        return (targetAngle)/steerLock;

}
float RecitingDriver::getAccel(CarState &cs)
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

void RecitingDriver::init(float *angles)
{
	g_learn_step_count = -1;
	g_reupdate_step_count = 0; //unnecessary, but good practice
	g_count = -1;
	g_learning_done = false;
	g_first_time = true;

	//Set Daniels datamembers
	mp_features = NULL;
	if (mp_Qinterface == NULL) {
		cout << "Creating LearningInterface...\n";
		try
		{
			mp_Qinterface = new LearningInterface();
			bool load_NN = true;

			ifstream is;
			is.open("RD_first_run_QNN");
			if( load_NN && is.is_open()) {
				is.close();
				cout << "\nLoading NN\n";
				mp_Qinterface->init("RD_first_run_QNN");
			}
			else
				mp_Qinterface->init();
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
	m_last_dist = 0;

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

double l_reward = 0;
CarControl RecitingDriver::wDrive(CarState cs)
{
	//keep track of time
	g_count++;

	// check if car is currently stuck
	if( (fabs(cs.getTrackPos()) > 0.9) && fabs(cs.getAngle()) >= 0.087266) //5 degrees
		stuck++; //if agent is driving on side of the track with nose pointed outwards, correct it.
	else if ( fabs(cs.getAngle()) > stuckAngle ) {
        stuck++; // update stuck counter
    } else {
        stuck = 0; // if not stuck reset stuck counter
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > stuckTime) {
    	/* set control, assuming car is pointing in a direction out of track */
		g_stuck_step_count++;
		mp_Qinterface->setEOE(true);
		l_reward -= g_stuck_penalty; //This value was chosen randomly!
    	CarControl cc =  carStuckControl(cs);
		//cc.setMeta(cc.META_RESTART); //NEW: Stuck means restart of race without penalty
		return cc;
    }

	//Do not use RL control if conditions are not right
	//For now: don't do anything before laptime == 0
	//*
	if(g_learning_done || cs.getCurLapTime() < 0){
		cout << "time: " << cs.getCurLapTime() << ". using script\n";
		return simpleBotControl(cs);
	}
	//*/

	//Car is not stuck:
	if(g_first_time) {
		g_first_time = false;
		g_count = 0;
	}

	//START LEARNING CODE
	
	if(g_count % g_steps_per_action == 0) 
	{
		//if( g_count % g_print_mod == 0)
		//	cout << "Penalty: "<< l_reward;

		//Compute reward of last action
		l_reward += computeReward(cs);
		//if( g_count % g_print_mod == 0)
		//	cout << "\tFinal Reward: " << l_reward <<endl;
		mp_Qinterface->setRewardPrevAction(l_reward);
		l_reward = 0;

		//do the actual learning step
		try{
			doLearning(cs);
		}catch (exception& e){
			cout << e.what() << endl;
			char end;
			cin >> end;
			exit(-3);
		}

		//get the driver's action
		mp_action_set = mp_Qinterface->getAction();
		if (mp_action_set == NULL) {
			cout << "Action is a NULL POINTER. Something went wrong.\n";
			char end;
			cin >> end;
			exit(-3);
		}

		//write info to log
		stringstream log;
		log << "time: " << g_count << "\tsteer: " << mp_action_set[0] << " accel: " << mp_action_set[1];
		mp_log->write(log.str());

		if(g_count % g_print_mod == 0)
			cout << "time: " << g_count << "\tsteer: " << mp_action_set[0] << " accel: " << mp_action_set[1] << endl;

	} else {
		mp_action_set = mp_Qinterface->getAction();
		if (cs.getCurLapTime() > 0 && g_count > g_steps_per_action) 
			//DE EERSTE g_steps_per_action STAPPEN VAN -->ELKE RONDE<-- UPDATE HIJ DUS NIET!!
		{
			//cout << "Driver: updating random old tuple\n";
			try{
				//cout << "g_count: " << g_count << "\t steps per action: " << g_steps_per_action << endl;
				//Waarom is g_count hier de eerste keer 50??
				g_reupdate_step_count = 0;
				while(g_reupdate_step_count < g_reupdate_steps_per_tick && !g_learning_done) {
					//Currently, these steps do not count for the max_steps of g_learning_done
					mp_Qinterface->updateWithOldTuple(LearningInterface::RANDOM);
					g_reupdate_step_count++;
				}
				
			}catch (exception& e){
				cout << e.what() << endl;
				char end;
				cin >> end;
				exit(-3);
			}
		}
		//cout << "Repeating action : " << mp_action_set[0] << "  " << mp_action_set[1] << endl;
		//cout << "repeating: "<< g_count % 50 << endl;
	}
	//END LEARNING CODE
	return rlControl(cs);
}

double RecitingDriver::computeReward(CarState &cs)
{
		double reward = 0;
		//
		///////////DISTANCE
		double distance = cs.getDistRaced();
		double dist_reward = 10* (distance - m_last_dist);
		m_last_dist = distance;

		if(g_count % g_print_mod == 0)
			cout << "Distance reward: "<< dist_reward << endl;
		reward+= dist_reward;

		///////////POSITION
		double pos_reward = abs(cs.getTrackPos());
		reward -= pos_reward;
		if(g_count % g_print_mod == 0)
			cout << "Position reward: -"<< pos_reward << endl;
		///////////DAMAGE
		//double damage_reward = -(cs.getDamage() - m_last_damage);
		////if( g_count % g_print_mod == 0) {
		////	cout << "   Damage reward: " << damage_reward;
		////}
		//m_last_damage = cs.getDamage();
		//reward += damage_reward;

		/////////OUTPUT
		//if( g_count % g_print_mod == 0)
		//	cout << "\tReward: " << reward;

		////

		//if(dist_reward > 0)
		//	reward = 2;
		//else
		//	reward = -2;

		//if(g_count != 0){
		//	if(abs(mp_action_set[0]) < 0.2)
		//		reward += 0.5;
		//	else
		//		reward -= 0.5;

		//	if(mp_action_set[1] > 0)
		//		reward += 1;
		//	else
		//		reward -= 1;
		//}
		
		//// Voor cacla: alleen in kosten denken. behalve optimale oplossing. oftewel kosten voor buiten range geen pos reward.
		if(g_count != 0)
			reward += mp_action_set[1];


		stringstream log;
		log << "time: " << g_count <<"\treward: " << reward << ". ";
		mp_log->write(log.str());
		stringstream rew_log;
		rew_log << reward;
		mp_reward_writer->write(rew_log.str());

		if(g_count % g_print_mod == 0)
			cout << endl << log.str() << endl;
		return reward;
}

void RecitingDriver::doLearning(CarState &cs) 
{
	if (g_count % (g_print_mod) == 0){
		//cout << "Time: " << g_count << ". ";
		cout << "\nLearn steps: " << g_learn_step_count  + g_reupdate_step_count << endl << endl;
	}
	//Get state features
	delete mp_features;
	mp_features = createFeatureVectorPointer(cs);
	//createFeatureVectorPointer(cs, mp_features); //misschien is het beter om een vector (pointer) mee te geven en deze te vullen?
	
	//Create a state
	mp_Qinterface->setState(mp_features);
	
	//Do some learning
	int l_learn_step_count = 0;
	while(l_learn_step_count < g_learn_steps_per_tick
			&& !g_learning_done){
		if (l_learn_step_count == 0)
			g_learning_done = mp_Qinterface->learningUpdateStep(true, LearningInterface::RANDOM);
		else
			g_learning_done = mp_Qinterface->learningUpdateStep(false, LearningInterface::RANDOM);
		l_learn_step_count++;
		g_learn_step_count++;
	}

	//write state to log
	//mp_Qinterface->logState(g_count);
	//log original action for debug purposes
	mp_Qinterface->logAction(g_count);

	//if end_of_episode was set due to stuck, it needs to be reset after the first learning step with eoe == true
	//if end_of_ep is set to false when agent realises that it isn't stuck, then there is no learning update with eoe
	if(mp_Qinterface->getEOE())
		mp_Qinterface->setEOE(false);

	if (g_learning_done){
		cout << "LEARNING IS DONE!\n";
		#ifdef WIN32
				char end;
				cin>>end;
		#endif
		exit(0);
	}
}

CarControl RecitingDriver::carStuckControl(CarState & cs)
{
	debug_stuck_count++;
	float acc = 0.8;
	// to bring car parallel to track axis
    float steer =  - cs.getAngle() / steerLock; 
    int gear=-1; // gear R
        
    // if car is pointing in the correct direction revert gear and steer  
    if (cs.getAngle()*cs.getTrackPos()>0)
    {
        gear = 1;
        steer = -steer;
		acc = 0.8;
    }

	//if agent is driving on side of the track with nose pointed outwards, correct it.
	if(cs.getTrackPos()> 0.9 && cs.getAngle() >= 0.087266f)
		steer -= 0.087266f; //5degrees
	if(cs.getTrackPos() < -0.9 && cs.getAngle() <= -0.087266f)
		steer += 0.087266f;
    // Calculate clutching
    clutching(cs,clutch);

    // build a CarControl variable and return it
    CarControl cc (acc,0.0,gear,steer,clutch);
	endOfRunCheck(cs, cc);
    return cc;
}

CarControl RecitingDriver::simpleBotControl(CarState &cs)
{
	// compute gear 
    int gear = getGear(cs);
	cout << "Using script to steer.\n";
    // compute accel/brake command
	float accel_and_brake = getAccel(cs);
	// compute steering
	float steer = getSteer(cs);
        
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
		brake = 0;
	}
	else
	{
		accel = 0;
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

CarControl RecitingDriver::rlControl(CarState &cs)
{
	//cout << "Time: " << cs.getCurLapTime() << endl;
	debug_rlcontrol_count++;
	// compute gear 
    int gear = getGear(cs);
	float steer = float(mp_action_set[0]);

    // set accel and brake from the joint accel/brake command 
    float accel,brake;
    if (mp_action_set[1]>0)
    {
        accel = mp_action_set[1];
        brake = 0;
    }
    else
    {
        accel = 0;
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

void RecitingDriver::endOfRunCheck(CarState &cs, CarControl &cc)
{
	//Check if user wants to restart
	if (getKeyboardInput() == 'r')
		cc.setMeta(cc.META_RESTART);

	//Dit is mogelijk baanspecifiek
	//if(g_count >= 20000 || cs.getCurLapTime() > 390.00) //20.000 ticks of 6.5 minuut game tijd
	if(g_learn_step_count + g_reupdate_step_count >= 5000) //(g_learn_step_count >= 400) //or (g_count >= 4000) //(g_learn_step_count + g_reupdate_step_count >= 5000)
	{
		cc.setMeta(cc.META_RESTART);
		g_count = 0;
		cout << "Learning steps during this run: " << g_learn_step_count << endl;
		cout << "RL Control steps during this run: " << debug_rlcontrol_count << endl;
		cout << "Stuck steps during this run: " << debug_stuck_count << endl;
		debug_stuck_count = 0;
		debug_rlcontrol_count = 0;
		g_learn_step_count = -1;
		g_experiment_count++;
		//cout << "Experiment nr: " << g_experiment_count;
		stringstream msg;
		msg << "Experiment nr: " << g_experiment_count;
		mp_log->write(msg.str());
		cout << msg;
	}

	if(g_experiment_count == 20) {
		cout << "\nExperiments done.\n";
		#ifdef WIN32
				char end;
				cin>>end;
		#endif
		exit(0);
	}
}

float RecitingDriver::filterABS(CarState &cs,float brake)
{
	// convert speed to m/s
	float speed = float(cs.getSpeedX() / 3.6);
	// when spedd lower than min speed for abs do nothing
    if (speed < absMinSpeed)
        return brake;
    
    // compute the speed of wheels in m/s
    float slip = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        slip += cs.getWheelSpinVel(i) * wheelRadius[i];
    }
    // slip is the difference between actual speed of car and average speed of wheels
    slip = speed - slip/4.0f;
    // when slip too high applu ABS
    if (slip > absSlip)
    {
        brake = brake - (slip - absSlip)/absRange;
    }
    
    // check brake is not negative, otherwise set it to zero
    if (brake<0)
    	return 0;
    else
    	return brake;
}

void RecitingDriver::onShutdown()
{
	cout << "Bye bye!" << endl;
	delete mp_features;
	delete mp_log;
	delete mp_reward_writer;
	delete mp_action_set;
	delete mp_Qinterface; 
}

void RecitingDriver::onRestart()
{
	//delete mp_features;
	//delete mp_action_set;
	//delete mp_Qinterface;
    cout << "Restarting the race!" << endl;
	g_learn_step_count = -1;
	delete mp_reward_writer;

	stringstream newfile;
	newfile << "log_files/Reciting_driver_rewards_" << g_experiment_count << ".txt";
	mp_reward_writer = new Writer(newfile.str());

	try{
		//mp_Qinterface = new LearningInterface();
		//mp_Qinterface->init();
	} catch(exception& e) {
		cout << e.what() << endl;
	}
}

void RecitingDriver::clutching(CarState &cs, float &clutch)
{
  double maxClutch = clutchMax;

  // Check if the current situation is the race start
  if (cs.getCurLapTime()<clutchDeltaTime  && stage==RACE && cs.getDistRaced()<clutchDeltaRaced)
    clutch = maxClutch;

  // Adjust the current value of the clutch
  if(clutch > 0)
  {
    double delta = clutchDelta;
    if (cs.getGear() < 2)
	{
      // Apply a stronger clutch output when the gear is one and the race is just started
	  delta /= 2;
      maxClutch *= clutchMaxModifier;
      if (cs.getCurLapTime() < clutchMaxTime)
        clutch = maxClutch;
	}

    // check clutch is not bigger than maximum values
	clutch = min(maxClutch,double(clutch));

	// if clutch is not at max value decrease it quite quickly
	if (clutch!=maxClutch)
	{
	  clutch -= delta;
	  clutch = max(0.0,double(clutch));
	}
	// if clutch is at max value decrease it very slowly
	else
		clutch -= clutchDec;
  }
}

char RecitingDriver::getKeyboardInput()
{
    if (_kbhit())
    {
        //_getch(); // edit : if you want to check the arrow-keys you must call getch twice because special-keys have two values
        return _getch();
    }
    return 0; // if no key is pressed
}