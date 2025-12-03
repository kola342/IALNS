//improved ALNS for first problem
//基本数据结构：链表

#include "HyperParameter.h"
#include "GlobalVariable.h"
#include "initialize.h"
#include "Functions.h"
#include "ImprovedALNS.h"
#include <iostream>
#include "cplex.h"

int main()
{
	double start, end;
	start = clock();
	initialGVariable();	//初始化全局变量并读取数据
	ImprovedALNS s_best = ImprovedALNS(initialSolution());//构造初始解
	cout << "初始解：" << s_best.getCost() << endl;
	ImprovedALNS s_current = s_best;
	int symbol_operators[3];	//记录选择的算子的索引
	int n = 0, seg = 0;
	double T;
	setT(T, s_best.getCost());
	double s_initial = s_best.getCost();
	int iteration = 0;
	int q;

	while (T >= T_END) {
		q = sigmoid(n);
		chooseOperator(symbol_operators);
		//cout << symbol_operators[0] << "---" << symbol_operators[1] << "---" << symbol_operators[2] << endl;
		ImprovedALNS s_new = s_current;
		s_new.removal_operation(q, symbol_operators[0]);
		s_new.noise_operation(symbol_operators[1]);
		int regret_k = -1;
		s_new.insertion_operation(q, symbol_operators[2], regret_k);
		s_new.intensifyOne();
		if (s_new.getCost() < s_best.getCost()) {
			s_best = s_new;
			s_current = s_new;
			n = 0;
			updatePoints(symbol_operators, DELTA1);
			updateWeightK(regret_k, DELTA1);//传入索引
			updateEdge(s_new, s_initial, s_current);
		}
		else {
			uniform_real_distribution<double> prob(0, 1);
			if (s_new.getCost() <= s_current.getCost()) {
				updatePoints(symbol_operators, DELTA2);
				updateWeightK(regret_k, DELTA2);//传入索引
				s_current = s_new;
				updateEdge(s_new, s_initial, s_current);
			}
			else if (prob(gen) <= exp((s_current.getCost() - s_new.getCost()) / T)) {
				updatePoints(symbol_operators, DELTA3);
				updateWeightK(regret_k, DELTA3);//传入索引
				s_current = s_new;
				updateEdge(s_new, s_initial, s_current);
			}
			else {
				updatePoints(symbol_operators, DELTA4);
				updateWeightK(regret_k, DELTA4);//传入索引
				updateEdge(s_new, s_initial, s_current);
			}
			n++;	
		}
		seg++;
		if (n==NOIMPRO) {
			s_current = s_best;
		}
		if (seg >= SEG) {
			updateWeight();	//更新算子的得分
			seg = 0;
			//s_new.output_weight('r');
		}
		iteration++;
		std::cout << iteration << "   ";
		std::cout << s_best.getCost() << endl;// << "---" << s_new.getCost() - s_best.getCost() << "---" << s_new.num_cust_truk() << endl;
		//cout << s_new.num_cust_truk() << endl;
		T = RATE_T * T;
	}
	end = clock();
	s_best.output();
	s_best.outputSolution();
	warmStart(s_best, 14400);
	std::cout << T << endl;
	std::cout << "运行时间：" << (end - start) / CLOCKS_PER_SEC << "s" << endl;
	return 0;
}