#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/control/MissileCrossTargetGuidance.hpp"
#include "../../include/analyzers/MissileFairZoneAnalyzer.hpp"
#include "../../include/utils/MyMath.hpp"
#include "../../include/aerodynamics/Atmosphere_GOST_4401_81.hpp"
#include <fstream>

using namespace std;

MissileCrossTargetGuidance::MissileCrossTargetGuidance():   K_guidance(vector<double>(2)), 
                                                            polynomCoefs(pair< vector<double>, vector<double> >
                                                            (vector<double>(4), vector<double>(4))){}

MissileCrossTargetGuidance::~MissileCrossTargetGuidance(){}

bool MissileCrossTargetGuidance::init(  vector<double>& _K_guidance, double _reGuidanceTime, double _dt){
    if(K_guidance.size() != _K_guidance.size()) 
        throw runtime_error("Wrong K_guidance vector size, while initialize CrossTargetGuidance");

    for(int i = 0; i < K_guidance.size(); i++){
        K_guidance[i] = _K_guidance[i];
    }
    time = 0;
    last_reGuidanceTime = numeric_limits<double>::min();
    reGuidanceTime = _reGuidanceTime;
    dt = _dt;
    needToUpdate = true;
    polynomCoefs = make_pair<vector<double>, vector<double>>({0,0,0,0}, {0,0,0,0});
    return true;
}

// Define the cubic polynomial y(x) = ax^3 + bx^2 + cx + d
double cubicPolynomial(double x, vector<double>& coeffs) {
    return coeffs[3] * x * x * x + coeffs[2] * x * x + coeffs[1] * x + coeffs[0];
}

// Define the derivative of the cubic polynomial y'(x)
double cubicPolynomialDerivative(double x, vector<double>& coeffs) {
    return 3 * coeffs[3] * x * x + 2 * coeffs[2] * x + coeffs[1];
}

double cubicPolynomialSecondDerivative(double x, vector<double>& coeffs) {
    return 6 * coeffs[3] * x + 2 * coeffs[2];
}

// Define the squared distance function
double distanceSquared(double x, double x_r, double y_r, vector<double>& coeffs) {
    double y = cubicPolynomial(x, coeffs);
    return (x - x_r) * (x - x_r) + (y - y_r) * (y - y_r);
}

// Perform gradient descent or numerical optimization to find the minimum
double findClosestPoint(double x_r, double y_r, vector<double>& coeffs, double tolerance = 1, int max_iterations = 10000) {
    double x = x_r; // Start with the x-coordinate of the given point as an initial guess

    for (int i = 0; i < max_iterations; ++i) {
        // Compute y(x) and its derivatives
        double y = cubicPolynomial(x, coeffs);
        double y_derivative = cubicPolynomialDerivative(x, coeffs);
        double y_second_derivative = cubicPolynomialSecondDerivative(x, coeffs);

        // Compute the gradient of the distance function
        double gradient = 2 * (x - x_r) + 2 * (y - y_r) * y_derivative;

        // Compute the second derivative (Hessian) of the distance function
        double hessian = 2 + 2 * (y_derivative * y_derivative) + 2 * (y - y_r) * y_second_derivative;

        // Newton's update step
        double new_x = x - gradient / hessian;

        // Check for convergence
        if (std::fabs(new_x - x) < tolerance) {
            return new_x;
        }

        x = new_x;
    }

    throw std::runtime_error("Newton's method did not converge within the maximum number of iterations.");
}

bool MissileCrossTargetGuidance::updateInformation(PointMass* missile, vector<PointMass*> targets){
    if(targets.size() != 2) 
        throw runtime_error("Try to use CrossTargetGuidance with targets number nonequal to 2!");
    time += dt;
    if(last_reGuidanceTime + reGuidanceTime < time){
        needToUpdate = true;
    }
    return true;
}

vector<double> MissileCrossTargetGuidance::currentMiss(PointMass* missile){
    vector<double> coefs_y = polynomCoefs.first;
    vector<double> r_missile = missile -> get_r();
    double x = findClosestPoint(r_missile[0], r_missile[1], coefs_y);
    double y_angle = angleBetweenVectors(r_missile[0], r_missile[1], x, cubicPolynomial(x, coefs_y));
    cout << "УГОЛ МЕЖДУ ТРАЕКТОРИЯМИ: " << y_angle << '\n';
    return {y_angle * missile -> get_Vabs() / Atmosphere_GOST_4401_81<double>::get_g(0), 0};
}

vector<double> MissileCrossTargetGuidance::get_GuidanceSignal(PointMass* missile, std::vector<PointMass*> targets){
    if(!updateInformation(missile, targets)) return {0, 0};
    vector<double> miss = currentMiss(missile);
    //ТУТ МОЖЕТ НАДО ДОБАВИТЬ УСЛОВИЙ...........................................................
    return {K_guidance[0] * miss[0], K_guidance[1] * miss[1]};
}

bool MissileCrossTargetGuidance::needToUpdateData(){
    return needToUpdate;
}

void MissileCrossTargetGuidance::updateData(pair< vector<double>, vector<double> >& data){
    cout <<"РАЗМЕР МАССИВА КОЭФФИЦИЕНТОВ: " << polynomCoefs.first.size() << '\n';
    ofstream out;          
    string name = "polynomCoefs" + to_string(int(data.first[0]))+ ".dat";
    for(size_t i = 0; i < polynomCoefs.first.size(); i ++){
        out.open(name, ios::app);
        out << data.first[i] << ' ';
        out.close();
        polynomCoefs.first[i] = data.first[i];
        polynomCoefs.second[i] = data.second[i];
    }
    last_reGuidanceTime = time;
    needToUpdate = false;
}