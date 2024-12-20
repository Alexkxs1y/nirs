#include <math.h>
#include <vector>
#include <stdexcept>
#include "../../include/simulations/MakeTargets.hpp"

using namespace std;


void makeOneTarget(Target& tag,vector<double> _state, TargetGuidance &tg){
    double m = 1;
    double n_max = 0;
    tag.init(m, _state, n_max, &tg);
}

void makeTargets(vector<Target> &tag, vector<TargetGuidance> &tg, int n, vector< vector<double> > _states){
    if(_states.size() != n) throw runtime_error("While creating n targets, size of _states doesnt equal n.");
    double m = 1;
    double n_max = 0;
    for(size_t i = 0; i < n; i++){
        tag[i].init(m, _states[i], n_max, &tg[i]);
    }
}