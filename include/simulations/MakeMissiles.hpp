#ifndef _MAKE_MISSILE_H
#define _MAKE_MISSILE_H

#include <math.h>
#include <vector>
#include "../models/Missile.hpp"

void makeOneMissile(Missile& mis,double _yaw, double _pitch, IAerodynamic & ma, MissileStabilization &ms, MissileGuidance &mg);

void makeMissiles(std::vector<Missile>& mis, std::vector<MissileGuidance> &mg, int n, std::vector<double>_yaw, std::vector<double>_pitch, IAerodynamic & ma, MissileStabilization &ms);

#endif