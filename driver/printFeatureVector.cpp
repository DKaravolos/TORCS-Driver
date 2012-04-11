#include "printFeatureVector.h"

using namespace std;

void printFeatureVector(vector<float> featureVector)
{
	cout << "Printing Features..."<< endl;
	int count = 0;
	for (vector<float>::iterator it = featureVector.begin(); it!=featureVector.end(); ++it){
		if (count < 4){
			cout << "Feature " << count << ": " << *it << endl;
		}
		else
			cout << " Feature " << count << " , range sensor: " << *it << endl;
		count++;
	}
	cout << "\n\n\n";
}

void printFeatureVectorPointer(vector<float>* featureVector)
{
	if(featureVector == NULL){ 
		cout << "Null pointer. Something is wrong!!" << endl;
		return;
	}
	
	cout << "Printing Features..."<< endl;
	int count = 0;
	for (vector<float>::iterator it = featureVector->begin(); it!=featureVector->end(); ++it){
		if (count < 4){
			cout << "Feature " << count << ": " << *it << endl;
		}
		else
			cout << " Feature " << count << " , range sensor: " << *it << endl;
		count++;
	}
	cout << "\n\n\n";
}