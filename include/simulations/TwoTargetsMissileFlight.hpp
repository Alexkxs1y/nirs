#ifndef _TWO_TARGETS_MISSILE_FLIGHT_H
#define _TWO_TARGETS_MISSILE_FLIGHT_H


#include <math.h>
#include <vector>
#include "../models/Missile.hpp"
#include "../models/Target.hpp"

std::vector<double> twoTargetsOneMissileFlight(Missile & missile, std::vector<Target> & targets, double dt);


#endif