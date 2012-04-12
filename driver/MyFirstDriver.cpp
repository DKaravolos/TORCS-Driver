/***************************************************************************
 
    file                 : MyFirstDriver.cpp
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "MyFirstDriver.h"

/* Gear Changing Constants*/
const int MyFirstDriver::gearUp[6]=
    {
        8000,9500,9500,9500,9500,0
    };
const int MyFirstDriver::gearDown[6]=
    {
        0,4000,6300,7000,7300,7300
    };

/*
const int MyFirstDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int MyFirstDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };
//*/

/* Stuck constants*/
const int MyFirstDriver::stuckTime = 25;
const float MyFirstDriver::stuckAngle = .523598775; //PI/6

/* Accel and Brake Constants*/
const float MyFirstDriver::maxSpeedDist=70;
const float MyFirstDriver::maxSpeed=150;
const float MyFirstDriver::sin5 = 0.08716;
const float MyFirstDriver::cos5 = 0.99619;

/* Steering constants*/
const float MyFirstDriver::steerLock=0.785398;
const float MyFirstDriver::steerSensitivityOffset=80.0;
const float MyFirstDriver::wheelSensitivityCoeff=1;

/* ABS Filter Constants */
const float MyFirstDriver::wheelRadius[4]={0.3179,0.3179,0.3276,0.3276};
const float MyFirstDriver::absSlip=2.0;
const float MyFirstDriver::absRange=3.0;
const float MyFirstDriver::absMinSpeed=3.0;

/* Clutch constants */
const float MyFirstDriver::clutchMax=0.5;
const float MyFirstDriver::clutchDelta=0.05;
const float MyFirstDriver::clutchRange=0.82;
const float MyFirstDriver::clutchDeltaTime=0.02;
const float MyFirstDriver::clutchDeltaRaced=10;
const float MyFirstDriver::clutchDec=0.01;
const float MyFirstDriver::clutchMaxModifier=1.3;
const float MyFirstDriver::clutchMaxTime=1.5;


int
MyFirstDriver::getGear(CarState &cs)
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

float
MyFirstDriver::getSteer(CarState &cs)
{
	// steering angle is compute by correcting the actual car angle w.r.t. to track 
	// axis [cs.getAngle()] and to adjust car position w.r.t to middle of track [cs.getTrackPos()*0.5]
    float targetAngle=(cs.getAngle()-cs.getTrackPos()*0.5);
    // at high speed reduce the steering command to avoid loosing the control
    if (cs.getSpeedX() > steerSensitivityOffset)
        return targetAngle/(steerLock*(cs.getSpeedX()-steerSensitivityOffset)*wheelSensitivityCoeff);
    else
        return (targetAngle)/steerLock;

}
float
MyFirstDriver::getAccel(CarState &cs)
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
        return 0.3; // when out of track returns a moderate acceleration command

}

int count = 0;
CarControl
MyFirstDriver::wDrive(CarState cs)
{
	//START LEARNING SEQUENCE
	delete dp_features;
	dp_features = createFeatureVectorPointer(cs); //misschien is het beter om een vector (pointer) mee te geven en deze te vullen?
	if( count++ % 50 == 0)
	{
		printFeatureVector(*dp_features);
		count = 0;
	}

	//Compute reward of last action
	int distance = cs.getDistRaced();
	dp_Qinterface->setRewardPrevAction(distance - d_last_dist);
	d_last_dist = distance;

	//Create a state
	dp_Qinterface->setState(dp_features);
	dp_Qinterface->printState();

	//Do some learning
	dp_Qinterface->experimentMainLoop();

	//get Action
	double* actionset = dp_Qinterface->getAction();
	if (actionset == NULL)
		cout << "Action is a NULL POINTER. Something went wrong.\n";

	//END LEARNING SEQUENCE

	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > stuckAngle )
    {
		// update stuck counter
        stuck++;
    }
    else
    {
    	// if not stuck reset stuck counter
        stuck = 0;
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > stuckTime)
    {
    	/* set gear and sterring command assuming car is 
    	 * pointing in a direction out of track */
    	
    	// to bring car parallel to track axis
        float steer = - cs.getAngle() / steerLock; 
        int gear=-1; // gear R
        
        // if car is pointing in the correct direction revert gear and steer  
        if (cs.getAngle()*cs.getTrackPos()>0)
        {
            gear = 1;
            steer = -steer;
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc (1.0,0.0,gear,steer,clutch);
        return cc;
    }

    else // car is not stuck
    {
    	// compute accel/brake command
        float accel_and_brake = getAccel(cs);
        // compute gear 
        int gear = getGear(cs);
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
        return cc;
    }
}

float
MyFirstDriver::filterABS(CarState &cs,float brake)
{
	// convert speed to m/s
	float speed = cs.getSpeedX() / 3.6;
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

void
MyFirstDriver::onShutdown()
{
	delete dp_features;
	delete dp_Qinterface;
    cout << "Bye bye!" << endl;
}

void
MyFirstDriver::onRestart()
{
	delete dp_features;
	delete dp_Qinterface;
    cout << "Restarting the race!" << endl;
}

void
MyFirstDriver::clutching(CarState &cs, float &clutch)
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

void
MyFirstDriver::init(float *angles)
{
	//Set Daniels datamembers
	dp_features = NULL;
	dp_Qinterface = new LearningInterface;
	d_last_dist = 0;

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