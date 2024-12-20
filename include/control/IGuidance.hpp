#ifndef IGUIDANCE_H
#define IGUIDANCE_H

#include <math.h>
#include <vector>
#include "../models/PointMass.hpp"

class IGuidance{
    public:
        virtual ~IGuidance() = default ;
        virtual std::vector<double> get_GuidanceSignal(PointMass* missile, std::vector< PointMass* > targets) = 0;
        virtual bool needToUpdateData() = 0;
        virtual void updateData(std::pair<std::vector<double>, std::vector<double>>& data) = 0;
};


#endif