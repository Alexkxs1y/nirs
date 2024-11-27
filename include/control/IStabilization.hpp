#ifndef ISTABILIZATION_H
#define ISTABILIZATION_H

#include <math.h>
#include <vector>
#include "../models/RigidBody.hpp"

class IStabilization{
    public:
        virtual ~IStabilization() = default ;
        virtual std::vector<double> get_controlParams(RigidBody* object, std::vector<double>& guidanceSignal) = 0;
};


#endif