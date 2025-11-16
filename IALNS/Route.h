#ifndef ROUTE_H
#define ROUTE_H
#include <list>

struct DroneRoute
{
	double time=0;//无人机路线的花费时间
	double total_demand=0;//无人机线路的需求量
	double electricity=0;//无人机路线已消耗的电量
	std::list<int> route;
	DroneRoute(double time, std::list<int> route) :time(time), route(route) {};
	DroneRoute(std::list<int> route) :route(route) {};
	DroneRoute(std::list<int> route, double demand, double e) :route(route), total_demand(demand), electricity(e) {};
};


struct Route
{
	int id=-1;
	double time_wait=0;
	double time_second=0;
	std::list<DroneRoute> routes;
	Route(int id, std::list<DroneRoute> routes) :id(id), routes(routes) {};
	Route(int id) :id(id) {};
};
#endif