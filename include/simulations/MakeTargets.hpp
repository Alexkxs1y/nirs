#ifndef _MAKE_TARGET_H
#define _MAKE_TARGET_H

#include <math.h>
#include <vector>
#include "../models/Target.hpp"

void makeOneTarget(Target& tag, std::vector<double> _state, TargetGuidance &tg);

void makeTargets(std::vector<Target>& tag, std::vector<TargetGuidance> &tg, int n, std::vector< std::vector<double> > _states);

#endif