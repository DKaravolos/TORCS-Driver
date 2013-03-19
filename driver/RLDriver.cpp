#include "RLDriver.h"
#ifndef WIN32
	#include <stdio.h>
	#include <termios.h>
	#include <unistd.h>
	#include <fcntl.h>
#endif

//#ifdef WIN32
//    #include <Windows.h>
//#endif
#define DRIVER_DEBUG true
#define DEF_TC_HAS_INFO true

#ifdef DEF_TC_HAS_INFO
	#include "../learning/TileCodingHM.h"
	#include "../learning/TCLearningInterface.h"
#endif

RLDriver::RLDriver()
{
	setPrefs();
	//Set user preferences

	bool test = false; //ALLEEN VOOR TEST DRIVER.exe

	if(test) //ALLEEN VOOR TEST DRIVER.exe
	{
		cout << "RLDriver: Using test settings!\n";
		m_round_size = 10000;
		m_exp_count = 1;
		m_save_nn = false;
	} else {
		cout << "How many updates should be in one round?\n";
		cin >> m_round_size;

		cout << "How many runs of "<< m_round_size << "? \n";
		cin >> m_exp_count;

		cout << "Do you want to save the data structure? (y/n)\n";
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
	m_automatic_experiment = false;
}

RLDriver::RLDriver(const int& nr_steps, const int& nr_runs, const bool& save_data)
{
	//Set user preferences
	setPrefs();
	m_round_size = nr_steps;
	m_exp_count = nr_runs;
	m_save_nn = save_data;
	
	if(m_save_nn)
		cout << "Saving nn.\n";
	else
		cout << "Not saving nn.\n";
	m_automatic_experiment = true;
}

RLDriver::~RLDriver()
{
	delete mp_features;
	//delete[] mp_action_set; //You don't need to delete this, because it points to mp_torcs_action in the RLInterface
	delete[] gp_prev_state;
}

void RLDriver::setPrefs()
{
	//Keep track of learning process
	g_learn_step_count = -1; //Deze begint op -1 omdat er in de eerste leerstap niets wordt geupdate. alleen reward geset.
	g_reupdate_step_count = 0;

	//Initialise counters and pointers to standard values
	stuck=0;clutch=0.0;
	mp_features = NULL;
	mp_RLinterface = NULL;
	gp_prev_state = NULL;

	g_steps_per_action = 1;
	g_print_mod = g_steps_per_action;
	g_learn_steps_per_tick = 1;
	g_reupdate_steps_per_tick = 0;

	cout << "Reupdate steps per unused time step: " << g_reupdate_steps_per_tick <<endl;
	g_stuck_penalty = 0;
	g_experiment_count = 0;

	debug_rlcontrol_count = 0;
}

void RLDriver::init(float *angles)
{
	g_count = 0;
	g_stuck_step_count = 0;
	g_learning_done = false;
	g_first_time = true;
	g_reupdates_left = 0;
	m_practice_saved = false;

	//Set debug variables
	debug_max_reward = 0;
	debug_max_speed = 0;
	debug_max_trackpos = 0;
	debug_max_angle = 0;
	debug_max_dist_front = 0;
	debug_max_distl40 = 0;
	debug_max_distl20 = 0;
	debug_max_distr20 = 0;
	debug_max_distr40 = 0;

	debug_min_reward = 10;
	debug_min_speed = 10;
	debug_min_trackpos = 10;
	debug_min_angle = 10;
	debug_min_dist_front = 10;
	debug_min_distl40 = 10;
	debug_min_distl20 = 10;
	debug_min_distr20 = 10;
	debug_min_distr40 = 10;

	//Set Daniels datamembers
	mp_features = new vector<double>;
	if (mp_RLinterface == NULL) {
		cout << "Creating LearningInterface...\n";
		try
		{
			initInterface(true, m_automatic_experiment);
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
		angles[i]= float(-90+i*15);
		angles[18-i]= float(90-i*15);
	}

	for (int i=5; i<9; i++)
	{
			angles[i]= float(-20+(i-5)*5);
			angles[18-i]= float(20-(i-5)*5);
	}
	angles[9]= 0.0f;
	cout << "LI steps = "<< mp_RLinterface->getSteps() << endl;
}

CarControl RLDriver::wDrive(CarState cs)
{
	//keep track of time
	g_count++;

	////// check if car is currently stuck
	if (stuckCheck(cs) && (g_count % g_steps_per_action == 0))
        stuck++; // update stuck counter
    else
        stuck = 0; // if not stuck reset stuck counter

	/*
	if(DRIVER_DEBUG && g_count % 10 == 0) {
	//	cout << "Distance from start: " << cs.getDistFromStart() << endl;
	//	cout << "Distance raced: "<< cs.getDistRaced() << endl;
	//	cout << "TrackPos: " << cs.getTrackPos() << "\n";
	//	cout << "Sensor left: " << cs.getTrack(5) << endl
	//	cout << "Sensor front: " << cs.getTrack(9) << endl;
	//	cout << "Sensor right: " << cs.getTrack(13) << endl;
	//}*/

	// after car is stuck for a while apply recovering policy //update: end the episode
	if  (
		((stuck > stuckTime) && 
		(fabs(cs.getSpeedX()) <= 10)) || // 29-08: changed from 5 to 30.
		(fabs(cs.getTrackPos()) > 0.95)
		)
	{ 
		g_stuck_step_count++;
		//cout << "Car stuck for too long. Ending Episode!\n";
		cout << "Car is stuck or outside of track. Ending episode!\n";
		if(getKeyboardInput() == 'p')
		{
			char temp;
			cin >> temp;
		}
		//The episode has ended, make sure the agent is properly updated.
		mp_RLinterface->setEOE();
		doLearning(cs, true);

		// Return a CarControl object with the restart command
    	CarControl cc =  carStuckControl(cs);
		cc.setMeta(cc.META_RESTART);
		return cc;
    }

	//Do not use RL control if conditions are not right
	//For now: don't do anything before laptime == 0
	//*
	if(g_learning_done || cs.getCurLapTime() < 0){
		//cout << "time: " << cs.getCurLapTime() << ". using script\n";
		return simpleBotControl(cs);
	}
	//*/

	//Reset the timer when official lap time is 0
	if(g_first_time) {
		g_first_time = false;
		g_count = 0;
	}

	//START LEARNING CODE
	
	if(g_count % g_steps_per_action == 0) 
	{
		//Do everything that involves learning.
		doLearning(cs, false);

		//get the driver's action
		mp_action_set = mp_RLinterface->getAction(); //update after computing reward, so it can be used as "last action" for computeReward
		if (mp_action_set == NULL) {
			cerr << "Action is a NULL POINTER. Something went wrong.\n";
			char end;
			cin >> end;
			exit(-3);
		}
		//cout << "Driver action - steer:" << mp_action_set[0] << "accel: " << mp_action_set[1] << endl;
		//Guard the update:reupdate ratio
		//g_reupdates_left += g_steps_per_action-1;

		////write info to log
		//stringstream log;
		//log << "time: " << g_count << "\tsteer: " << mp_action_set[0] << " accel: " << mp_action_set[1];
		//mp_log->write(log.str());

	} /*
	//else if (g_reupdates_left == 0) {
		//It seems that stuckControl has ended before a g_count % 10 == 0.
		//We do not want to perform updates until a learn step has been done (i.e. g_count % 10 == 0 again),
		//because then we will lose our fixed update:reupdate ratio.
		//But our current time tracking does not allow us to perform a learn step,
		//so instead of making time tracking more adaptive, we do script controlled steps
		//g_stuck_step_count++;
  //  	CarControl cc =  simpleBotControl(cs);
		//return cc;
		//mp_action_set = mp_RLinterface->getAction(); //Repeat previous action without reupdate
	//} else {
	//	--g_reupdates_left;
	//	//cout << "Time: "<< g_count << ". Reupdate step. Reupdates left: " << g_reupdates_left << endl;

	//	mp_action_set = mp_RLinterface->getAction();
	//	if (cs.getCurLapTime() > 0 && g_count > g_steps_per_action) 
	//		//De eerste g_steps_per_action stappen van elke ronde reupdate hij niet, omdat de eerste actie een random actie was.
	//		//De state na eerste actie wordt gebruikt als beginstate, omdat er dan pas een vorige state + actie is.
	//		//Daarom staat de learning_count ook eerst op -1. Na de eerste function call wordt deze op 0 gezet,
	//		//waardoor hij na de eerste update pas op 1 staat.
	//		//Het is dus ook niet logisch om in de tussentijd al wel te reupdaten.
	//		//Als gevolg hiervan staan er dus 1+g_steps_per_action (10) rlcontrol acties die niet
	//		//in de learning_count of reupdate_count terug komen. Dat is dus verklaarbaar, logisch en gewenst.
	//	{
			//cout << "Driver: updating random old tuple\n";
			//try{
			//	//cout << "g_count: " << g_count << "\t steps per action: " << g_steps_per_action << endl;
			//	int l_reupdate_step_count = 0;
			//	while(l_reupdate_step_count < g_reupdate_steps_per_tick && !g_learning_done) {
			//		//Currently, these steps do not count for the max_steps of g_learning_done
			//		mp_RLinterface->updateWithOldTuple(RLInterface::RANDOM);
			//		++g_reupdate_step_count;
			//		++l_reupdate_step_count;
			//	}
			//	
			//}catch (exception& e){
			//	cout << e.what() << endl;
			//	char end;
			//	cin >> end;
			//	exit(-3);
			//}
	//	}
	//	//cout << "Repeating action : " << mp_action_set[0] << "  " << mp_action_set[1] << endl;
	//	//cout << "repeating: "<< g_count % 50 << endl;
	//}
	//END LEARNING CODE
	*/
	return rlControl(cs);
}

bool RLDriver::stuckCheck(CarState& cs)
{
	////if agent is driving on side of the track with nose pointed outwards
	//if	( (cs.getTrackPos() >  0.8 && cs.getAngle() >= 2 * 0.087266) || // angle > 10 degrees
	//	  (cs.getTrackPos() < -0.8 && cs.getAngle() <= 2 * -0.087266)
	//	) 
	//	return true; 

	////if agent is almost outside of track and does not move
	//if (cs.getSpeedX() < 5 && (cs.getTrackPos() >  0.99 || cs.getTrackPos() <  -0.99))
	//	 return true;
	
	//if agent's angle is larger than predefined stuckAngle
	return (fabs(cs.getAngle()) > stuckAngle); //last condition, so we can return its value. if it is true: stuck, if false: not stuck.
}

double RLDriver::computeReward(CarState &state, double* action, CarState &next_state, bool end_of_ep)
{
	if(g_count % 10 == 0)
		cout << "Time: " << g_count << ".\t";
	//double[2] action is not used for computing the reward
	double reward = 0;
		
	/////////DISTANCE
	int dist_weight = 1;
	double dist_reward = next_state.getDistRaced() - state.getDistRaced();

	if(next_state.getCurLapTime() < state.getCurLapTime()){ //detect end of lap
		dist_reward = next_state.getDistFromStart(); // otherwise, reward is negative
		stringstream msg;
		//msg << "End of lap at " << state.getCurLapTime()
		//	<< "\tTotal number of steps: " << g_learn_step_count;
			////<< "\nEnd of lap reward: " << dist_reward;
		//mp_log->write(msg.str());
		msg << state.getCurLapTime();
		mp_lap_writer->write(msg.str());
	}
	reward+= dist_weight * dist_reward; 
	//if(DRIVER_DEBUG)
		//cout << "\tDistance reward: "<< dist_reward << ".\n";

	if( dist_reward < 0.002)
		reward -= 1;

	///////// END OF EPISODE
	if(end_of_ep){
		reward -= 2;
		stringstream msg;
		//msg << "End of episode at " << state.getCurLapTime()
		//	<< "\tTotal number of steps: " << g_learn_step_count;
		//mp_log->write(msg.str());
		msg << state.getCurLapTime();
		mp_eoe_writer->write(msg.str());
	}
	////// EXTRA INFO
	if(DRIVER_DEBUG && g_count % 10 == 0)
		printf("Reward = %-8.2f",reward);
		//cout << "\tFinal reward: " << setw(8) << left << setprecision(4) << reward << ". ";

	if(reward > debug_max_reward) //Keep track of max reward
		debug_max_reward = reward;
	if(reward < debug_min_reward) //Keep track of min reward
		debug_min_reward = reward;

	return reward;
}

//doLearning does everything that is involved with learning
void RLDriver::doLearning(CarState &cs, bool end_of_ep)
{
	//Compute reward of last action
	double l_reward;
	if(gp_prev_state != NULL)
	{
		l_reward = computeReward(*gp_prev_state, mp_action_set, cs, end_of_ep);
		if(mp_reward_writer != NULL)
		{
			stringstream ss;
			ss << l_reward;
			mp_reward_writer->write(ss.str());
		}
	}else{
		l_reward = 0;
		cerr << "gp_prev_state is NULL. l_reward is zero.\n";
	}
	
	//Pass reward to the interface
	mp_RLinterface->setRewardPrevAction(l_reward);

	//Get state features
	//createFeatureVectorPointer(cs, mp_features); //creates 13 features
	createSmallFeatureVectorPointer(cs, mp_features); //creates 8 features

	//Check if input is not out of bounds
	checkSensorInput(mp_features);
	//Create a state (pass state features to the interface)
	mp_RLinterface->setState(mp_features, g_count);
	
	if(DEF_TC_HAS_INFO){ // TC heuristic is based on actual CarState. If heuristic is used, give CarState to TileCodingHM.
		TCLearningInterface* l_TCLI = static_cast<TCLearningInterface*>(mp_RLinterface); 
		*(l_TCLI->public_car_control) = simpleBotControl(cs);
		*(l_TCLI->public_car_state) = cs;
	}
	//do the actual update step
	try{
		doUpdate(cs);
	}catch (exception& e){
		cout << e.what() << endl;
		char end;
		cin >> end;
		exit(-3);
	}

	//Save current state as prev_state for computing reward next state
	if (gp_prev_state == NULL)
		gp_prev_state = new CarState();
	*gp_prev_state = cs;
}

//doUpdate only does the update(s) through the interface
void RLDriver::doUpdate(CarState &cs) 
{
	if (g_count % (10) == 0){ //g_print_mod
		//cout << "Time: " << g_count << ". ";
		cout << "Total update steps: " << g_learn_step_count  + g_reupdate_step_count << endl;
	}

	//Do as much updates as predefined
	int l_learn_step_count = 0;
	while (l_learn_step_count < g_learn_steps_per_tick
			&& !g_learning_done){
		if (l_learn_step_count == 0)
			//g_learning_done = mp_RLinterface->learningUpdateStep(true, RLInterface::TD); 
			g_learning_done = mp_RLinterface->learningUpdateStep(true, RLInterface::RANDOM); // We do store tuples at the moment
		else
			g_learning_done = mp_RLinterface->learningUpdateStep(false, RLInterface::RANDOM);
		l_learn_step_count++;
		g_learn_step_count++;
	}

	//write state to log
	//mp_RLinterface->logState(g_count);
	//log original action for debug purposes
	//mp_RLinterface->logAction(g_count);

	if (g_learning_done){ //Old check. Probably doesn't do anything anymore
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

	if (getKeyboardInput() == 's')
	{
		int l_id = m_network_id*1000 + g_learn_step_count;
		mp_RLinterface->writeNetwork(l_id, g_learn_step_count);
	}

	//Extra save network possibility. To save network while continuing to learn.
	//Assumes one run of X updates and no reupdates
	if (g_learn_step_count % 150000 == 0 && !m_practice_saved)
	{
		mp_log->write("Saving network at 150.000 updates");
		int l_id = m_network_id*1000 + g_learn_step_count;
		mp_RLinterface->writeNetwork(l_id, g_learn_step_count);
		m_practice_saved = true;
	}

	stringstream debug_msg;

	if(g_learn_step_count >= 10 && g_learn_step_count + g_reupdate_step_count >= m_round_size) //waarom moet learn_step_count groter zijn dan 10?
	{
		cc.setMeta(cc.META_RESTART);
		g_count = 0;
		//cout << "Learning steps during this run: " << g_learn_step_count << endl;
		
		debug_msg << "steps in LI: " << mp_RLinterface->getSteps() << endl;
		debug_msg << "learn steps: " << g_learn_step_count << "\treupdate: "<< g_reupdate_step_count << endl; //was debug_msg
		debug_msg << "rl_control: " << debug_rlcontrol_count << endl;
		debug_msg << "Maximal reward: " << debug_max_reward << endl;
		debug_msg << "Minimal reward: " << debug_min_reward << endl;
		mp_log->write(debug_msg.str());
		
		g_experiment_count++;		
		cout << "Experiment nr: " << g_experiment_count;
		printInputRange(g_experiment_count);
		//When should the network be saved?
		//Save first, last and every couple of runs
		if(g_experiment_count == m_exp_count) //g_experiment_count == 1 || g_experiment_count % 5 == 0 || 
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
	if(g_experiment_count == m_exp_count && !m_automatic_experiment) {
		cout << "\nExperiments done.\n";
		//onShutdown(); //this only calls RLDriver::onShutdown(), the actual driver does not shut down, so this is hardly useful
		cout << "MAX REWARD WAS: "<< debug_max_reward << endl;
		cout << "MIN REWARD WAS: "<< debug_min_reward << endl;
		printInputRange(g_experiment_count);

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
	//cout << "Time: "<< g_count << ". Using script to steer.\n";
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
        accel = float(mp_action_set[1]);
        brake = 0.0f;
    }
    else
    {
        accel = 0.0f;
        // apply ABS to brake
        brake = filterABS(cs,float(-mp_action_set[1]));
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
	//delete mp_features;
	//mp_features = NULL;
	//delete mp_action_set;
	//delete gp_prev_state;
}

void RLDriver::onRestart() 
{
	//delete mp_features;
	mp_features = NULL;
	//delete mp_RLinterface; // We are not reinitializing the interface between runs. This may have negative side-effects
	mp_RLinterface->setFirstTime(true); // one of the side-effects is having to manually set first time
    cout << "Restarting the race!" << endl;
	//g_learn_step_count = -1; // I'm trying to keep counting between restarts

	gp_prev_state = NULL;

	//Check parameter settings
	//cout << "Epsilon: " << mp_RLinterface->getExperiment()->getEpsilon() << endl;
	//cout << "Gamma: " << mp_RLinterface->getExperiment()->getGamma() << endl;
	//cout << "Learning Rate: " << mp_RLinterface->getExperiment()->getLearningRate() << endl;
	//cout << "Tau: " << mp_RLinterface->getExperiment()->getTau() << endl;
}
void RLDriver::clutching(CarState &cs, float &clutch)
{
  float maxClutch = clutchMax;

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
	clutch = min(maxClutch,clutch);

	// if clutch is not at max value decrease it quite quickly
	if (clutch!=maxClutch)
	{
	  clutch -= float(delta);
	  clutch = max(0.0f,clutch);
	}
	// if clutch is at max value decrease it very slowly
	else
		clutch -= clutchDec;
  }
}


#ifndef WIN32
int _kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

inline int _getch() {
	return getchar();
}
#endif

char RLDriver::getKeyboardInput()
{
	//#ifdef WIN32 //sorry, function is not implemented for linux -> now it is!
    if (_kbhit())
    {
        //_getch(); // edit : if you want to check the arrow-keys you must call getch twice because special-keys have two values
        return _getch();
    }
    return 0; // if no key is pressed
//#endif
}

char RLDriver::getArrowInput()
{
//#ifdef WIN32 //sorry, function is not implemented for linux -> now it is!
    if (_kbhit())
    {
        _getch(); // edit : if you want to check the arrow-keys you must call getch twice because special-keys have two values
        return _getch();
    }
    return 0; // if no key is pressed
//#endif
}

//Functions for automatic experiments
void RLDriver::changeLogWriterTo(string& new_file)
{
	delete mp_log;
	mp_log = new Writer(new_file);
}

void RLDriver::changeRewardWriterTo(string& new_file)
{
	delete mp_reward_writer;
	mp_reward_writer = new Writer(new_file);
}

int RLDriver::getLearningStep()
{
	int curr_step = g_experiment_count * m_round_size;
	curr_step += g_learn_step_count + g_reupdate_step_count;

	return curr_step;
}

void RLDriver::checkSensorInput(vector<double>* input)
{
	//check speed
	if(input->at(0) > debug_max_speed)
		debug_max_speed = input->at(0);
	if(input->at(0) < debug_min_speed)
		debug_min_speed = input->at(0);

	//check trackpos
	if(input->at(1) > debug_max_trackpos)
		debug_max_trackpos = input->at(1);
	if(input->at(1) < debug_min_trackpos)
		debug_min_trackpos = input->at(1);
	
	if(input->size() == 3)
	{
		//check front sensor
		if(input->at(2) > debug_max_dist_front)
			debug_max_dist_front = input->at(2);
		if(input->at(2) < debug_min_dist_front)
			debug_min_dist_front = input->at(2);

	} else if(input->size() == 8)
	{
		//check angle
		if(input->at(2) > debug_max_angle)
			debug_max_angle = input->at(2);
		if(input->at(2) < debug_min_angle)
			debug_min_angle = input->at(2);

		//check dist left 40
		if(input->at(3) > debug_max_distl40)
			debug_max_distl40 = input->at(3);
		if(input->at(3) < debug_min_distl40)
			debug_min_distl40 = input->at(3);

		//check dist left 20
		if(input->at(4) > debug_max_distl20)
			debug_max_distl20 = input->at(4);
		if(input->at(4) < debug_min_distl20)
			debug_min_distl20 = input->at(4);

		//check front sensor
		if(input->at(5) > debug_max_dist_front)
			debug_max_dist_front = input->at(5);
		if(input->at(5) < debug_min_dist_front)
			debug_min_dist_front = input->at(5);

		//check dist right 20
		if(input->at(6) > debug_max_distr20)
			debug_max_distr20 = input->at(6);
		if(input->at(6) < debug_min_distr20)
			debug_min_distr20 = input->at(6);

		//check dist right 40
		if(input->at(7) > debug_max_distr40)
			debug_max_distr40 = input->at(7);
		if(input->at(7) < debug_min_distr40)
			debug_min_distr40 = input->at(7);

	} else {
		cerr << "WARNING: UNKNOWN INPUT! Number of dimensions: " << input->size() << endl;
	}
}

void RLDriver::printInputRange(int l_experiment_count)
{
	stringstream file_out;
	file_out << "Input range of experiment " << l_experiment_count << endl;
	file_out << "Range of reward. \tMin: "<< debug_min_reward << " \t\tMax: " << debug_max_reward << endl; 
	file_out << "Range of speed. \tMin: "<< debug_min_speed << " \tMax: " << debug_max_speed << endl;
	file_out << "Range of trackPos. \tMin: "<< debug_min_trackpos << " \t\tMax: " << debug_max_trackpos << endl;
	file_out << "Range of front sensor. \tMin: "<< debug_min_dist_front << " \t\tMax: " << debug_max_dist_front << endl;

	if(mp_features->size() == 8)
	{
		file_out << "Range of angle. \tMin: "<< debug_min_angle << " \tMax: " << debug_max_angle << endl;
		file_out << "Range of sensor  40. \tMin: "<< debug_min_distl40 << " \t\tMax: " << debug_max_distl40 << endl;
		file_out << "Range of sensor  20. \tMin: "<< debug_min_distl20 << " \t\tMax: " << debug_max_distl20 << endl;
		file_out << "Range of sensor -20. \tMin: "<< debug_min_distr20 << " \t\tMax: " << debug_max_distr20 << endl;
		file_out << "Range of sensor -40. \tMin: "<< debug_min_distr40 << " \t\tMax: " << debug_max_distr40 << endl;
	}
	file_out << endl;
	mp_log->write(file_out.str());
}
