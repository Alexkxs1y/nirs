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

double squareDistance(const vector<double> &a, const vector<double> &b){
    return (a[0] - b[0]) * (a[0] - b[0]) + 
           (a[1] - b[1]) * (a[1] - b[1]) + 
           (a[2] - b[2]) * (a[2] - b[2]);
}

vector<double> nearesPointFromSample(const vector<double> &a, const vector<double> &b, vector< vector<double> >& sample){
    int nearestID = -1;
    double minDistance = numeric_limits<double>::max();
    double distance = 0;
    for(size_t i = 0; i < sample.size(); i ++){
        distance += squareDistance(a, sample[i]) + squareDistance(b, sample[i]);
        if(distance < minDistance){
            nearestID = i;
            minDistance = distance;
        }
        distance = 0;
    }
    return sample[nearestID];
}

void normalize(vector<double> &a){
    double norm = 0;
    for(size_t i = 0; i < a.size(); i ++){
        norm += a[i] * a[i];
    }
    norm = sqrt(norm);
    for(size_t i = 0; i < a.size(); i ++){
        a[i] /= norm;
    }
    return;
}

vector<double> crossProduct(const vector<double>& a, const vector<double>& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

double dotProduct(const vector<double>& a, const vector<double>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

//Поворот вектора a относительно вектора b на угол Theta 
vector<double> rotate(const vector<double>& a, const vector<double>& b, double Theta){
    vector<double> res(a.size());
    double a_Dot_b = dotProduct(a, b);
    double _cos = cos(Theta);
    double _sin = sin(Theta);
    vector<double> b_Cross_a = crossProduct(b, a);
    for(size_t i = 0; i < res.size(); i++){
        res[i] = a[i] * _cos +  b_Cross_a[i] * _sin + b[i] * a_Dot_b * (1 - _cos);
    }
    return res;
}