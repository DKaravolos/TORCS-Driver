#include "createFeatureVector.h"

using namespace std;

vector<float> createFeatureVector(CarState state)
{
	vector<float> featureVector;
	featureVector.push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.
	featureVector.push_back(state.getSpeedX()/300);//Genormaliseerd. 300kmh lijkt max. praktisch bijna niet boven 290.
	featureVector.push_back(state.getSpeedY()); // Geen idee wat we hiermee kunnen qua normalisatie
	featureVector.push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track. 
	for(int i = 0; i <= 8; i++){
		featureVector.push_back(state.getTrack(5+i)/200); //200m is de max_range
	}
	return featureVector;
}

vector<float>* createFeatureVectorPointer(CarState state)
{
	vector<float>* featureVector = new vector<float>;
	featureVector->push_back(state.getAngle()/PI); // Genormaliseerd. Range is tussen -pi en pi.
	featureVector->push_back(state.getSpeedX()/300);// Genormaliseerd. 300kmh lijkt max. praktisch bijna niet boven 290.
	featureVector->push_back(state.getSpeedY()); // Geen idee wat we hiermee kunnen qua normalisatie
	featureVector->push_back(state.getTrackPos()); //Zit al tussen -1 (rechts) en 1 (links). Waarde buiten deze range betekent off-track.
	for(int i = 0; i < 10; i++){
		featureVector->push_back(state.getTrack(5+i)/200); //Genormaliseerd. 200m is de max_range
	}
	return featureVector;
}