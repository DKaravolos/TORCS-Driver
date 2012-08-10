#ifndef CREATE_FEATURE_VECTOR_H
#define CREATE_FEATURE_VECTOR_H
#include "..\driver\CarState.h"
#include <vector>
#define PI 3.14159265

vector<double> createFeatureVector(const CarState state);
vector<double>* createFeatureVectorPointer(const CarState state);
void createFeatureVectorPointer(const CarState state, vector<double>* featureVector);
void createSmallFeatureVectorPointer(const CarState state, vector<double>* featureVector);

#endif //CREATE_FEATURE_VECTOR_H