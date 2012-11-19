#include "createFeatureVector.h"

using namespace std;
/*
vector<double> createFeatureVector(CarState state)
{
	vector<double> featureVector;
	featureVector.push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.
	//featureVector.push_back(state.getSpeedX()/300);//Genormaliseerd. 300kmh lijkt max. praktisch bijna niet boven 290.
	bool speed_x = state.getSpeedX() > 70;
	featureVector.push_back(double(speed_x));
	
	//featureVector.push_back(state.getSpeedY()); // Geen idee wat we hiermee kunnen qua normalisatie
	bool speed_y = state.getSpeedY() > 0.5;
	featureVector.push_back(double(speed_y));

	featureVector.push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track. 
	for(int i = 0; i <= 8; i++){
		featureVector.push_back(state.getTrack(5+i)/200); //200m is de max_range
	}
	return featureVector;
}
*/
/*
vector<double>* createFeatureVectorPointer(CarState state)
{
	vector<double>* featureVector = new vector<double>;
	featureVector->push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.

	//bool speed_x = state.getSpeedX() > 70;
	featureVector->push_back(state.getSpeedX());
	
	//bool speed_y = state.getSpeedY() > 0.5;
	featureVector->push_back(state.getSpeedY());

	featureVector->push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track.
	for(int i = 0; i < 10; i++)
		featureVector->push_back(state.getTrack(5+i) ); //Is er een rand dichterbij 2m of niet: double(state.getTrack(5+i)>2)
	return featureVector;
}
//*/

//*
vector<double>* createFeatureVectorPointer(CarState state)
{
	vector<double>* featureVector = new vector<double>;
	featureVector->push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.
	featureVector->push_back(state.getSpeedX()/300);// Genormaliseerd. 300kmh lijkt max. praktisch bijna niet boven 290.
	featureVector->push_back(state.getSpeedY()); // Geen idee wat we hiermee kunnen qua normalisatie
	featureVector->push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track.
	for(int i = 0; i < 9; i++){
		featureVector->push_back(state.getTrack(5+i)/200); //Genormaliseerd. 200m is de max_range
	}
	return featureVector;
}
//0 = sensor 5 = 40 degrees (left?)
//1 - 6 = 30
//2 - 7 = 20
//3 - 8 = 10

//4 - 9 = 0
//
//5 - 10 = -10
//6 - 11 = -20
//7 - 12 = -30
//8 - 13 = -40

//*/

void createFeatureVectorPointer(CarState state, vector<double>* featureVector)
{
	featureVector->clear();
	featureVector->push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.
	featureVector->push_back(state.getSpeedX()/300);// Genormaliseerd. 300kmh lijkt max. praktisch bijna niet boven 290.
	featureVector->push_back(state.getSpeedY()); // Geen idee wat we hiermee kunnen qua normalisatie
	featureVector->push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track.
	for(int i = 0; i < 10; i++){
		featureVector->push_back(state.getTrack(5+i)/200); //Genormaliseerd. 200m is de max_range
	}
}

void createSmallFeatureVectorPointer(CarState state, vector<double>* featureVector)
{
	featureVector->clear();
	featureVector->push_back(state.getSpeedX()/300);// Genormaliseerd. 300kmh lijkt max. praktisch bijna niet boven 290.
	featureVector->push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track (maar bestaat dus wel!!)
	featureVector->push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.

	double left_20 = (state.getTrack(6) + state.getTrack(7) + state.getTrack(8))/3;
	double right_20 = (state.getTrack(10) + state.getTrack(11) + state.getTrack(12))/3;
	
	featureVector->push_back(state.getTrack(5)/200); // -40 graden
	featureVector->push_back(left_20/200); // -20 graden gemiddeld.
	featureVector->push_back(state.getTrack(9)/200); // 0 graden
	featureVector->push_back(right_20/200); // 20 graden gemiddeld
	featureVector->push_back(state.getTrack(13)/200); //40 graden
}

void createMinimalFeatureVectorPointer(CarState state, vector<double>* featureVector)
{
	featureVector->clear();
	featureVector->push_back(state.getSpeedX()/300);// Genormaliseerd. 300kmh lijkt max. praktisch bijna niet boven 290.
	featureVector->push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track (maar bestaat dus wel!!)
	featureVector->push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.

	featureVector->push_back(state.getTrack(9)/200); // 0 graden
}
