#ifndef _MY_MATH_H
#define _MY_MATH_H

#include <math.h>
#include <vector>
#include <string>


std::vector<double> sub(std::vector<double> &a, std::vector<double> &b);

double range(std::vector<double>& a, std::vector<double>& b);

std::vector<double> nearesPointFromSample(const std::vector<double> &a, const std::vector<double> &b, const std::vector< std::vector<double> >& sample);

double squareDistance(const std::vector<double> &a, const std::vector<double> &b);

void normalize(std::vector<double> &a);

std::vector<double> rotate(const vector<double>& a, const vector<double>& b, double Theta);



#endif