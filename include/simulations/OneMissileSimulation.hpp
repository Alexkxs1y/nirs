#ifndef _ONE_MISSILE_SIMULATION_H
#define _ONE_MISSILE_SIMULATION_H


#include <math.h>
#include <vector>
#include "../models/Missile.hpp"
#include "../models/Target.hpp"

std::vector<double> oneMissileSimulation(Missile* missile, Target* target, double dt);


#endif