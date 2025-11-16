#ifndef CPLEX_H
#define CPLEX_H
#include "ImprovedALNS.h"

struct Satelite {
	Route satelite;
	int num = 0;
	list<int> cust;
	double cost;
	Satelite(Route a, int b, list<int> c, double d) :satelite(a), num(b), cust(c), cost(d) {};
};

extern list<Satelite> memory;
void intensifyTwo(int, ImprovedALNS&, ImprovedALNS&);
void warmStart(ImprovedALNS&, int);

#endif