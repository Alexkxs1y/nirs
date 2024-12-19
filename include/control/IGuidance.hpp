#ifndef IGUIDANCE_H
#define IGUIDANCE_H

#include <math.h>
#include <vector>
#include "../models/PointMass.hpp"

class IGuidance{
    public:
        virtual ~IGuidance() = default ;
        virtual std::vector<double> get_GuidanceSignal(PointMass* missile, std::vector<PointMass*> targets) = 0;
};


#endif