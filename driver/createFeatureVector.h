#pragma once
#include "CarState.h"
#include <vector>
#define PI 3.14159265

vector<float> createFeatureVector(const CarState state);
vector<float>* createFeatureVectorPointer(const CarState state);
