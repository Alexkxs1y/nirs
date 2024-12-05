#include <stdexcept>
#include <math.h>
#include <vector>
#include <string>
#include "../../include/utils/MyMath.hpp"

using namespace std;

vector<double> sub(vector<double>& a, vector<double>& b){
    if(a.size() != b.size()){
        throw std::runtime_error("Exception in MyMath. Calc sub between differen size vectors!\n");
    }
    vector<double> res(a.size());

    for(int i = 0; i < a.size(); i++){
        res[i] = a[i] - b[i];
    }
    return res;
}

double range(vector<double> &a, vector<double>& b){
    if(a.size() != b.size()){
        throw std::runtime_error("Exception in MyMath. Calc range between differen size vectors!\n");
    }
    double range = 0;
    for(int i = 0; i < a.size(); i++){
        range += (b[i] - a[i]) * (b[i] - a[i]);
    }
    return sqrt(range);
}