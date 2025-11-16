#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include "ImprovedALNS.h"

int sigmoid(const int&);
void chooseOperator(int*);
void updateEdge(ImprovedALNS&, const double, ImprovedALNS&);
void updateWeight();
void updatePoints(int*, double);
void setT(double&, double);
void updateWeightK(int, double);

#endif
