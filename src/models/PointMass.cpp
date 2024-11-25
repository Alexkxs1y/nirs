#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/PointMass.hpp"

using namespace std;

PointMass::PointMass(): stateVector(vector<double>(6)), forces(vector<double>(3)), forcesUpToDate(false){}

//PointMass::