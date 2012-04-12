#include "..\..\rlcpp\world.h"
#include "LearningInterface.h"
#include <vector>

class TorcsWorld : public World
{
public:
	TorcsWorld(LearningInterface* interface_pointer);
	~TorcsWorld(void);

	double act( Action * ); //returns reward

private:
	//datamembers
	State* dp_state; //wie gooit dit weg?
	Action* dp_action; //wie gooit dit weg?
	bool d_end_of_ep;
	
	LearningInterface* dp_torcs_interface;

	//functions:
	void initState();

//inline functions
public:
	inline void TorcsWorld::setState(State* state)	{ dp_state = state;}
	inline void TorcsWorld::getState(State* state)	{ state = dp_state;}
	inline Action* TorcsWorld::getAction()			{ return dp_action;}
	inline const char * TorcsWorld::getName()		{ return "TorcsWorld";}
	inline void setEOE(bool new_val)				{d_end_of_ep = new_val;}
	inline bool endOfEpisode()						{return d_end_of_ep;}
};
