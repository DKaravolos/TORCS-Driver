#ifndef TORCSWORLD_H
#define TORCSWORLD_H

#include <iostream>
#include <vector>
#include <algorithm>

#include "..\rlcpp\world.h"
#include "..\rlcpp\State.h"
#include "..\rlcpp\Action.h"


class TorcsWorld : public World
{
public:
	enum Configuration { QLEARNING, CACLA, BAS, QOS5, QOS3, QSTEER5, QSTEER3};
	TorcsWorld();
	TorcsWorld(Configuration);
	~TorcsWorld(void);

	double act( Action * ); //DEPRECATED. returns reward
	void convertDiscreteAction(Action* action, double* torcs_action);
	void convertContinuousAction(Action* action, double* torcs_action);

private:
	//datamembers
	State* mp_state; //learning interface maakt dit aan en gooit dit weg (check dit)
	Action* mp_action; //learning interface maakt dit aan en gooit dit weg (check dit)
	double m_reward;
	bool m_end_of_ep;
	Configuration m_config;
	
	//functions:
	//void initState();

//inline functions
public:
	inline void TorcsWorld::setState(State* state)	{mp_state = state;}
	inline void TorcsWorld::getState(State* state)	{state = mp_state;}

	inline Action* TorcsWorld::getAction()			{return mp_action;}

	inline void setReward(double reward)			{m_reward = reward;}
	inline double getReward()						{return m_reward;}
	
	inline const char * TorcsWorld::getName()		{return "TorcsWorld";}
	inline void setEOE(bool new_val)				{m_end_of_ep = new_val;}
	inline bool endOfEpisode()						{return m_end_of_ep;}
};

#endif //TORCSWORLD_H