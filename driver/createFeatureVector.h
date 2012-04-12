#pragma once
#include "CarState.h"
#include <vector>
#define PI 3.14159265

vector<double> createFeatureVector(const CarState state);
vector<double>* createFeatureVectorPointer(const CarState state);
