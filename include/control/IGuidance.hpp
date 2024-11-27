#ifndef IGUIDANCE_H
#define IGUIDANCE_H

#include <math.h>
#include <vector>
#include "../models/PointMass.hpp"

class GuidanceSystem{
    public:
        virtual ~GuidanceSystem() = default ;
        virtual std::vector<double> get_GuidanceSignal(PointMass* missile, PointMass* target) const = 0;
};


#endif