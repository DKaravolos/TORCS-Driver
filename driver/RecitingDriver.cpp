#include "RecitingDriver.h"

unsigned int debug_stuck_count = 0;
unsigned int debug_learn_count = 0;

RecitingDriver::RecitingDriver()
{
	stuck=0;clutch=0.0;
	mp_features = NULL;
	mp_Qinterface = NULL;
	m_last_dist = 0;
	m_last_dist_from_start = 0;
	m_last_damage = 0;

	g_print_mod = 20;
	g_steps_per_action = 20;
	g_learn_steps_per_tick = 1;
	g_stuck_penalty = 10;
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
	g_learn_step_count = 0;
	g_count = -1;
	g_learning_done = false;
	g_first_time = true;

	//Set Daniels datamembers
	mp_features = NULL;
	if (mp_Qinterface == NULL) {
		cout << "Creating LearningInterface...\n";
		mp_Qinterface = new LearningInterface();
		cout << "mp_Qinterface is now at " << mp_Qinterface << endl;
		mp_Qinterface->init();
		cout << "Done.\n";
	} else {
		cout << "Already created a LearningInterface. Skipping constructor and init.\n";
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

	// if (angle is <20 degrees or trackpos <10% from center) : not stuck anymore
	//if (stuck >= stuckTime && ((fabs(cs.getAngle()) < 0.34f) || fabs(cs.getTrackPos()) < 0.05)) { 
		//cout << "unstuck :)" <<endl;
        //stuck = 0; // update stuck counter
   // }	

	if( (fabs(cs.getTrackPos()) > 0.9) && fabs(cs.getAngle()) >= 0.087266)
		stuck++; //if agent is driving on side of the track with nose pointed outwards, correct it.
	else if ( fabs(cs.getAngle()) > stuckAngle ) {
        stuck++; // update stuck counter
    }else {
		if(stuck>= stuckTime)
			cout << "unstuck :)" <<endl;
        stuck = 0; // if not stuck reset stuck counter
		
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > stuckTime) {
    	/* set control, assuming car is pointing in a direction out of track */
		cout << "Stuck! :(" << endl;
		g_stuck_step_count++;
		mp_Qinterface->setEOE(true);
		l_reward -= g_stuck_penalty; //This value was chosen completely random!
    	return carStuckControl(cs);
    }
	//Car is not stuck:
	if(g_first_time) { ///////HO, nu zijn er twee checks voor first_time. de andere zit in de update van LI
		g_learning_done = mp_Qinterface->learningUpdateStep(false); //Niet opslaan. Er is immers geen reward
		g_first_time = false;
	}
	//START LEARNING CODE
	if(g_count % g_steps_per_action == 0) 
	{
		if( g_count % g_print_mod == 0)
			cout << "Penalty: "<< l_reward;
		//Compute reward of last action
		l_reward += computeReward(cs);
		if( g_count % g_print_mod == 0)
			cout << "\tFinal Reward: " << l_reward <<endl;
		mp_Qinterface->setRewardPrevAction(l_reward);
		l_reward = 0;
		//do the actual learning step
		try{
			doLearning(cs);
		}catch (exception& e){
			cout << e.what() << endl;
			char end;
			cin >> end;
			exit(-1);
		}
		//get the driver's action
		mp_action_set = mp_Qinterface->getAction();
		if (mp_action_set == NULL) {
			cout << "Action is a NULL POINTER. Something went wrong.\n";
			exit(-1);
		}

	} else {
		if (g_count > g_steps_per_action) //DE EERSTE g_steps_per_action STAPPEN VAN -->ELKE EPISODE<-- UPDATE HIJ DUS NIET!!
		{
			//cout << "Driver: updating random old tuple\n";
			try{
				mp_Qinterface->updateWithOldTuple(LearningInterface::RANDOM);
			}catch (exception& e){
				cout << e.what() << endl;
				char end;
				cin >> end;
				exit(-1);
			}
		}
		//cout << "Repeating action : " << mp_action_set[0] << "  " << mp_action_set[1] << endl;
		//cout << "repeating: "<< g_count % 50 << endl;
	}
	//END LEARNING CODE
	//*
	if(g_learning_done)
		return simpleBotControl(cs);
	//*
	return rlControl(cs);
}

double RecitingDriver::computeReward(CarState &cs)
{
		double reward = 0;
		
		/////////DISTANCE
		double distance = cs.getDistRaced();
		double dist_reward = 10* (distance - m_last_dist);
		m_last_dist = distance;
		//if( g_count % g_print_mod == 0) { 
		//	cout << "Distance reward: "<< dist_reward;
		//}
		reward+= dist_reward;

		/////////DAMAGE
		double damage_reward = -(cs.getDamage() - m_last_damage);
		//if( g_count % g_print_mod == 0) {
		//	cout << "   Damage reward: " << damage_reward;
		//}
		m_last_damage = cs.getDamage();
		reward += damage_reward;

		/////////OUTPUT
		if( g_count % g_print_mod == 0)
			cout << "\tReward: " << reward;
		return reward;
}

void RecitingDriver::doLearning(CarState &cs) 
{
	//Get state features
	delete mp_features;
	mp_features = createFeatureVectorPointer(cs);
	//createFeatureVectorPointer(cs, mp_features); //misschien is het beter om een vector (pointer) mee te geven en deze te vullen?
	//Create a state
	mp_Qinterface->setState(mp_features);
	
	//Do some learning
	g_learn_step_count = 0;
	while(g_count > g_learn_steps_per_tick && g_learn_step_count < g_learn_steps_per_tick){
		/*if(g_learn_step_count ==0)
			cout << "\nlearning "<< g_learn_steps_per_tick << "times:";*/
		g_learning_done = mp_Qinterface->learningUpdateStep(true);
		g_learn_step_count++;
	}
	//if end_of_episode was set due to stuck, it needs to be reset after the first learning step with eoe == true
	//if end_of_ep is set to false when agent realises that it isn't stuck, then there is no learning update with eoe
	if(mp_Qinterface->getEOE())
		mp_Qinterface->setEOE(false);

	//if (g_learning_done)
	//	cout << "LEARNING IS DONE! (i'm doing nothing with this information, though)\n";
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
	if(g_count >= 20000) { //Dit is mogelijk baanspecifiek
		cc.setMeta(cc.META_RESTART);
		g_count = 0;
		cout << "Learning steps during this run: " << debug_learn_count << endl;
		cout << "Stuck steps during this run: " << debug_stuck_count << endl;
		debug_stuck_count = 0;
		debug_learn_count = 0;
	}
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
	if(g_count >= 20000) { //Dit is mogelijk baanspecifiek
		cc.setMeta(cc.META_RESTART);
		g_count = 0;
		cout << " YOU'RE DOING IT WRONG!!!!" << endl;
	}
	return cc;
}

CarControl RecitingDriver::rlControl(CarState &cs)
{
	debug_learn_count++;
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
	if(g_count >= 20000) { //Dit is mogelijk baanspecifiek
		cc.setMeta(cc.META_RESTART);
		g_count = 0;
		cout << "Learning steps during this run: " << debug_learn_count << endl;
		cout << "Stuck steps during this run: " << debug_stuck_count << endl;
		debug_stuck_count = 0;
		debug_learn_count = 0;
	}

    return cc;
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
	//delete mp_action_set;
	//delete mp_Qinterface; 
}

void RecitingDriver::onRestart()
{
	//delete mp_features;
	//delete mp_action_set;
	//delete mp_Qinterface;
    cout << "Restarting the race!" << endl;
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