#include "RLDriver.h"
//#ifdef WIN32
//    #include <Windows.h>
//#endif

RLDriver::RLDriver()
{
	
	//Keep track of learning process
	g_learn_step_count = -1; //Deze begint op -1 omdat er in de eerste leerstap niets wordt geupdate. alleen reward geset.
	g_reupdate_step_count = 0;

	//Initialise counters and pointers to standard values
	stuck=0;clutch=0.0;
	mp_features = NULL;
	mp_RLinterface = NULL;
	gp_prev_state = NULL;

	g_steps_per_action = 10;
	g_print_mod = g_steps_per_action;
	g_learn_steps_per_tick = 1;
	g_reupdate_steps_per_tick = 1;

	g_stuck_penalty = 0;
	g_experiment_count = 0;

	debug_rlcontrol_count = 0;

	//Set user preferences
	cout << "How many updates should be in one round?\n";
	cin >> m_round_size;

	cout << "How many runs of "<< m_round_size << "? \n";
	cin >> m_exp_count;

	cout << "Do you want to save the neural network? (y/n)\n";
	char answer = 'n';
	cin >> answer;
	if(answer == 'y')
	{
		cout << "Saving nn.\n";
		m_save_nn = true;
	} else {
		cout << "Not saving nn.\n";
		m_save_nn = false;
	}
}

void RLDriver::init(float *angles)
{
	g_count = 0;
	g_learning_done = false;
	g_first_time = true;
	g_reupdates_left = 0;

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
	cout << "LI steps = "<< mp_RLinterface->getSteps() << endl;
}

CarControl RLDriver::wDrive(CarState cs)
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
	if ((stuck > stuckTime) && (cs.getSpeedX() <= 5)) {
    	/* set control, assuming car is pointing in a direction out of track */
		g_stuck_step_count++;
		mp_RLinterface->setEOE();
    	CarControl cc =  carStuckControl(cs);

		if(g_stuck_step_count > 1000 & g_stuck_step_count > 2 * g_learn_step_count)
		{
			cerr << "\n\nTHE CAR HAS BEEN STUCK FOR TOO MANY TIME STEPS! RESTART!\n\n";
			cc.setMeta(cc.META_RESTART);
			stringstream msg;
			msg << "Time: "<< g_count << ". Agent has been stuck for too long."
				<< "Stuck count: " <<  g_stuck_step_count
				<< "\tLearn step count (this run): " <<  g_learn_step_count;
			mp_log->write(msg.str());
		} else {
			stringstream msg;
			msg << "Time: "<< g_count << ". Stuck count: " <<  g_stuck_step_count
				<< "\tLearn step count (this run): " <<  g_learn_step_count;
			mp_log->write(msg.str());
		}
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
		//cout << "Time: "<< g_count << ". Learn step.\n";
		
		//Guard the update:reupdate ratio
		g_reupdates_left += 9; //I suppose you could let the user choose this number or at least see it as a parameter

		//Compute reward of last action
		double l_reward;
		if(gp_prev_state != NULL)
		{
			l_reward = computeReward(*gp_prev_state, mp_action_set, cs);
			stringstream ss;
			ss << l_reward;
			mp_reward_writer->write(ss.str());
		}else{
			l_reward = 0;
			cerr << "gp_prev_state is NULL. l_reward is zero.\n";
		}
		
		//if( g_count % g_print_mod == 0)
		//	cout << "\tFinal Reward: " << l_c_reward <<endl;
		mp_RLinterface->setRewardPrevAction(l_reward);

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
		mp_action_set = mp_RLinterface->getAction(); //update after computing reward, so it can be used as "last action" for computeReward
		if (mp_action_set == NULL) {
			cerr << "Action is a NULL POINTER. Something went wrong.\n";
			char end;
			cin >> end;
			exit(-3);
		}

		//Save current state as prev_state for computing reward next state
		if (gp_prev_state == NULL)
			gp_prev_state = new CarState();
		*gp_prev_state = cs;

		////write info to log
		//stringstream log;
		//log << "time: " << g_count << "\tsteer: " << mp_action_set[0] << " accel: " << mp_action_set[1];
		//mp_log->write(log.str());
		//cout << "time: " << g_count << "\tsteer: " << mp_action_set[0] << " accel: " << mp_action_set[1];

	} else if (g_reupdates_left == 0) {
		//It seems that stuckControl has ended before a g_count % 10 == 0.
		//We do not want to perform updates until a learn step has been done (i.e. g_count % 10 == 0 again),
		//because then we will lose our fixed update:reupdate ratio.
		//But our current time tracking does not allow us to perform a learn step,
		//so instead of making time tracking more adaptive, we do script controlled steps
		//g_stuck_step_count++;
  //  	CarControl cc =  simpleBotControl(cs);
		//return cc;
		mp_action_set = mp_RLinterface->getAction(); //Repeat previous action without reupdate
	} else {
		--g_reupdates_left;
		//cout << "Time: "<< g_count << ". Reupdate step. Reupdates left: " << g_reupdates_left << endl;

		mp_action_set = mp_RLinterface->getAction();
		if (cs.getCurLapTime() > 0 && g_count > g_steps_per_action) 
			//De eerste g_steps_per_action stappen van elke ronde reupdate hij niet, omdat de eerste actie een random actie was.
			//De state na eerste actie wordt gebruikt als beginstate, omdat er dan pas een vorige state + actie is.
			//Daarom staat de learning_count ook eerst op -1. Na de eerste function call wordt deze op 0 gezet,
			//waardoor hij na de eerste update pas op 1 staat.
			//Het is dus ook niet logisch om in de tussentijd al wel te reupdaten.
			//Als gevolg hiervan staan er dus 1+g_steps_per_action (10) rlcontrol acties die niet
			//in de learning_count of reupdate_count terug komen. Dat is dus verklaarbaar, logisch en gewenst.
		{
			//cout << "Driver: updating random old tuple\n";
			try{
				//cout << "g_count: " << g_count << "\t steps per action: " << g_steps_per_action << endl;
				int l_reupdate_step_count = 0;
				while(l_reupdate_step_count < g_reupdate_steps_per_tick && !g_learning_done) {
					//Currently, these steps do not count for the max_steps of g_learning_done
					mp_RLinterface->updateWithOldTuple(RLInterface::TD);
					++g_reupdate_step_count;
					++l_reupdate_step_count;
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

double RLDriver::computeReward(CarState &state, double* action, CarState &next_state)
{
	//double[2] action is not used for computing the reward
	double reward = 0;
		
	/////////DISTANCE
	double dist_reward = next_state.getDistRaced() - state.getDistRaced();
	//cout << "Distance reward: "<< dist_reward <<endl;
	reward+= dist_reward;

	if(next_state.getSpeedX() <= 10) //Drive, damn you!!
		reward -= 2;
	if(next_state.getSpeedX() <= 5)
		reward -= 2;
	if(next_state.getSpeedX() >= 10)
		reward += 2;

	///////////POSITION
	double pos_reward = 0;
	if(abs(next_state.getTrackPos() > 0.1)) // Dit was 0.5 voor QOS
		pos_reward = -2* abs(next_state.getTrackPos());
	
	if (abs(next_state.getTrackPos() > 0.8))
		pos_reward = -10;
	
	
	reward += pos_reward;
	
	///////////DAMAGE
	//double damage_reward = -(next_state.getDamage() - state.getDamage());
	//cout << "Damage reward: " << damage_reward << endl;
	//reward += damage_reward;

	/////////ACTION
	//if(g_count != 0)
	//	reward += action[1];

	//cout << "time: " << g_count <<"\treward: " << reward << ". ";

	return reward;
}

void RLDriver::doLearning(CarState &cs) 
{
	if (g_count % (g_print_mod) == 0){
		cout << "Time: " << g_count << ". ";
		cout << "\tLearn + Reupdate steps: " << g_learn_step_count  + g_reupdate_step_count << endl;
	}
	//Get state features
	//createFeatureVectorPointer(cs, mp_features); //creates 13 features
	createSmallFeatureVectorPointer(cs, mp_features); //creates 7 features
	//Create a state
	mp_RLinterface->setState(mp_features);
	
	//Do some learning
	int l_learn_step_count = 0;
	while(	l_learn_step_count < g_learn_steps_per_tick
			&& !g_learning_done){
		if (l_learn_step_count == 0)
			//g_learning_done = mp_RLinterface->learningUpdateStep(true, RLInterface::TD); 
			g_learning_done = mp_RLinterface->learningUpdateStep(true, RLInterface::TD); // We do store tuples at the moment
		else
			g_learning_done = mp_RLinterface->learningUpdateStep(false, RLInterface::TD);
		l_learn_step_count++;
		g_learn_step_count++;
	}

	//write state to log
	//mp_RLinterface->logState(g_count);
	//log original action for debug purposes
	//mp_RLinterface->logAction(g_count);

	if (g_learning_done){
		cout << "LEARNING IS DONE!\n";
		#ifdef WIN32
				char end;
				cin>>end;
		#endif
		exit(0);
	}
}

void RLDriver::endOfRunCheck(CarState &cs, CarControl &cc)
{
	//Check if user wants to restart
	if (getKeyboardInput() == 'r')
		cc.setMeta(cc.META_RESTART);

	stringstream debug_msg;
	if(g_learn_step_count >= 10 && g_learn_step_count + g_reupdate_step_count == m_round_size) // or (g_count >= 4000) ?
	{
		cc.setMeta(cc.META_RESTART);
		g_count = 0;
		//cout << "Learning steps during this run: " << g_learn_step_count << endl;
		
		debug_msg << "steps in LI: " << mp_RLinterface->getSteps() << endl;
		debug_msg << "learn steps: " << g_learn_step_count << "\treupdate: "<< g_reupdate_step_count << endl; //was debug_msg
		debug_msg << "rl_control: " << debug_rlcontrol_count << endl;
		mp_log->write(debug_msg.str());
		
		g_experiment_count++;		
		cout << "Experiment nr: " << g_experiment_count;

		//When should the network be saved?
		//Save first, last and every couple of runs
		if(g_experiment_count == 1 || g_experiment_count % 5 == 0 || g_experiment_count == m_exp_count)
		{
			int l_id = m_network_id*1000 + g_experiment_count * (m_round_size);
			if(m_save_nn) //Only save NN if the user wants to.
			{
				cout << "\nSaving Network\n";
				mp_RLinterface->writeNetwork(l_id, g_experiment_count *(g_learn_step_count + g_reupdate_step_count));
			} else {
				cout << "NOT SAVING NETWORK!\n";
			}
		}

		//Het is handig om door te tellen bij restarts entoch een simpele endOfRunCheck te kunnen doen.
		//Nu weten we zeker dat de ronde afgelopen is, dus nu moeten de tellertjes pas weer op 0 gezet worden.
		g_learn_step_count = -1; //deze begint op -1 omdat er in de eerste leerstap niets wordt geupdate. alleen reward geset.
		g_reupdate_step_count = 0;
	}

	//Experiments are done when the # of experiments is equal to m_exp_count (that was defined by the user).
	if(g_experiment_count == m_exp_count) {
		cout << "\nExperiments done.\n";
		#ifdef WIN32
				char end;
				cin>>end;
		#endif
		exit(0);
	}
}

CarControl RLDriver::carStuckControl(CarState & cs)
{
	float acc = 0.8f;
	// to bring car parallel to track axis
    float steer =  - cs.getAngle() / steerLock; 
    int gear=-1; // gear R
        
    // if car is pointing in the correct direction revert gear and steer  
    if (cs.getAngle()*cs.getTrackPos()>0)
    {
        gear = 1;
        steer = -steer;
		acc = 0.8f;
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

CarControl RLDriver::simpleBotControl(CarState &cs)
{
	// compute gear 
    int gear = getGear(cs);
	cout << "Time: "<< g_count << ". Using script to steer.\n";
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

CarControl RLDriver::rlControl(CarState &cs)
{
	debug_rlcontrol_count++;
	//cout << "Time: " << cs.getCurLapTime() << endl;
	// compute gear 
    int gear = getGear(cs);
	float steer = float(mp_action_set[0]);

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

/* Gear Changing Constants*/
const int RLDriver::gearUp[6]=
    {
        8000,9500,9500,9500,9500,0
    };
const int RLDriver::gearDown[6]=
    {
        0,4000,6300,7000,7300,7300
    };

/*
const int RLDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int RLDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };
//*/

/* Stuck constants*/
const int RLDriver::stuckTime = 25;
const float RLDriver::stuckAngle = 0.785398163f; //.523598775f; //PI/6

/* Accel and Brake Constants*/
const float RLDriver::maxSpeedDist=70;
const float RLDriver::maxSpeed=150;
const float RLDriver::sin5 = 0.08716f;
const float RLDriver::cos5 = 0.99619f;

/* Steering constants*/
const float RLDriver::steerLock=0.785398f;
//const float RLDriver::steerSensitivityOffset=80.0f;
//const float RLDriver::wheelSensitivityCoeff=1;
const float RLDriver::steerSensitivityOffset=100.0f;
const float RLDriver::wheelSensitivityCoeff=0.5;

/* ABS Filter Constants */
const float RLDriver::wheelRadius[4]={0.3179f,0.3179f,0.3276f,0.3276f};
const float RLDriver::absSlip=2.0f;
const float RLDriver::absRange=3.0f;
const float RLDriver::absMinSpeed=3.0f;

/* Clutch constants */
const float RLDriver::clutchMax=0.5f;
const float RLDriver::clutchDelta=0.05f;
const float RLDriver::clutchRange=0.82f;
const float RLDriver::clutchDeltaTime=0.02f;
const float RLDriver::clutchDeltaRaced=10;
const float RLDriver::clutchDec=0.01f;
const float RLDriver::clutchMaxModifier=1.3f;
const float RLDriver::clutchMaxTime=1.5f;



int RLDriver::getGear(CarState &cs)
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

float RLDriver::getSteer(CarState &cs)
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

float RLDriver::getAccel(CarState &cs)
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

float RLDriver::filterABS(CarState &cs,float brake)
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

void RLDriver::onShutdown()
{
	cout << "Bye bye!" << endl;
	delete mp_features;
	mp_features = NULL;
	delete mp_log;
	delete mp_reward_writer;
	delete mp_RLinterface; 
	delete gp_prev_state;
}

void RLDriver::clutching(CarState &cs, float &clutch)
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

char RLDriver::getKeyboardInput()
{
#ifdef WIN32 //sorry, function is not implemented for linux
    if (_kbhit())
    {
        //_getch(); // edit : if you want to check the arrow-keys you must call getch twice because special-keys have two values
        return _getch();
    }
    return 0; // if no key is pressed
#endif

}
