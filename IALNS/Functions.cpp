#include "Functions.h"
#include "HyperParameter.h"
#include "GlobalVariable.h"
#include <random>
#include <iostream>
#include <cmath>


int sigmoid(const int& noImproved) {
	double gamma = (double)NUM_REMOVAL_MAX / NUM_REMOVAL_MIN;
	double q = (1 - std::exp(ALPHA * (BETA - noImproved))) / (1 + std::exp(ALPHA * (BETA - noImproved)));
	q = q + (gamma + 1) / (gamma - 1);
	q = (gamma - 1) / (2 * gamma) * NUM_REMOVAL_MAX * q;
	return (int)std::ceil(q);
}

void chooseOperator(int* symbols) {
	std::uniform_real_distribution<double> prob1(0, 1);
	double p = prob1(gen);
	double sum = 0;
	for (size_t i = 0; i < kWeightRemoval.size(); i++) {
		sum += kWeightRemoval[i];
	}
	double t = 0;
	symbols[0] = 0;
	for (size_t i = 0; i < kWeightRemoval.size(); i++) {
		t += kWeightRemoval[i] / sum;
		if (p > t)
			symbols[0] = i + 1;
		else
			break;
	}

	//同上
	std::uniform_real_distribution<double> prob2(0, 1);
	if (prob2(gen) <= kWeightNoise[0] / (kWeightNoise[0] + kWeightNoise[1]))
		symbols[1] = 0;
	else
		symbols[1] = 1;

	//同上
	std::uniform_real_distribution<double> prob3(0, 1);
	p = prob3(gen);
	sum = 0;
	for (size_t i = 0; i < kWeightInsertion.size(); i++) {
		sum += kWeightInsertion[i];
	}
	t = 0;
	symbols[2] = 0;
	for (int i = 0; i < 3; i++) {
		t += kWeightInsertion[i] / sum;
		if (p > t)
			symbols[2] = i + 1;
		else
			break;
	}
}

void updateEdge(ImprovedALNS& s_new, const double s_initial, ImprovedALNS& s_current) {
	s_new.updateEdge(EDGE_K*s_initial / s_new.getCost());
}

void updateWeight() {
	for (size_t i = 0; i < kWeightInsertion.size(); i++) {
		if (points_i[i][1] != 0) {
			kWeightInsertion[i] = kWeightInsertion[i] * (1 - R) + R * points_i[i][0] / points_i[i][1];
		}
	}
	for (size_t i = 0; i < kWeightNoise.size(); i++) {
		if (points_n[i][1] != 0) {			
			kWeightNoise[i] = kWeightNoise[i] * (1 - R) + R * points_n[i][0] / points_n[i][1];
		}
	}
	for (size_t i = 0; i < kWeightRemoval.size(); i++) {
		if (points_r[i][1] != 0) {
			kWeightRemoval[i] = kWeightRemoval[i] * (1 - R) + R * points_r[i][0] / points_r[i][1];
		}
	}
}

void updatePoints(int* symbol_operators, double DELTA) {
	points_r[symbol_operators[0]][0] += DELTA;
	points_r[symbol_operators[0]][1]++;
	points_n[symbol_operators[1]][0] += DELTA;
	points_n[symbol_operators[1]][1]++;
	points_i[symbol_operators[2]][0] += DELTA;
	points_i[symbol_operators[2]][1]++;
}

void setT(double& T, double cost) {
	T = - cost * W / log(0.5);
	cout << "初始温度：" << T << endl;
}

void updateWeightK(int k, double delta) {//传入索引
	if (k < 0)
		return;
	weightK[k] = weightK[k] * (1 - R);
	weightK[k] += (delta * R);
}
