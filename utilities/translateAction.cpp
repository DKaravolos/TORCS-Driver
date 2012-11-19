#include "translateAction.h"

std::string translateAction(int action)
{
	std::string translation;
	switch(action)
	{
		case 0:
			translation = "Straight, Accelerate.";
			break;

		case 1:
			translation = "Straight, Neutral.";		
			break;

		case 2:
			translation = "Straight, Brake.";
			break;

		case 3:
			translation = "Full right, Accelerate.";
			break;

		case 4:
			translation = "Full right, Neutral.";
			break;

		case 5:
			translation = "Full right, Brake.";
			break;

		case 6:
			translation = "Full left, Accelerate.";
			break;
		case 7:
			translation = "Full left, Neutral.";
			break;

		case 8:
			translation = "Full left, Brake.";
			break;
		//steer half right
		case 9:
			translation = "Half right, Accelerate.";
			break;

		case 10:
			translation = "Half right, Neutral.";
			break;

		case 11:
			translation = "Half right, Neutral.";
			break;
		//steer half left
		case 12:
			translation = "Half left, Accelerate.";
			break;
		case 13:
			translation = "Half left, Neutral.";
			break;

		case 14:
			translation = "Half left, Brake.";
			break;
	}
	return translation;
}