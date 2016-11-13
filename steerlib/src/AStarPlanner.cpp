//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define HEURISTIC_WEIGHT 1
#define DIAGONAL_COST 1

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface  * _gSpatialDatabase, bool append_to_path)
	{
		
		gSpatialDatabase = _gSpatialDatabase;
		std::set<int> openlist_astar;
		std::set<int> openlist_wastar;
		std::set<int> openlist_other;
		std::set<int> closedlist_astar;
		std::set<int> closedlist_wastar;
		std::set<int> closedlist_other;

		OpenList.push_back(openlist_astar);
		OpenList.push_back(openlist_wastar);
		OpenList.push_back(openlist_other);
		ClosedList.push_back(closedlist_astar);
		ClosedList.push_back(closedlist_wastar);
		ClosedList.push_back(closedlist_other);
		//Initialize f and g map scores
		std::map<int, AStarPlannerNode*> nodeMap;
		for (int i = 0; i < gSpatialDatabase->getNumCellsX(); ++i) {
			for (int j = 0; j < gSpatialDatabase->getNumCellsZ(); ++j) {
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				nodeMap[index] = new AStarPlannerNode(getPointFromGridIndex(index), (double)INFINITY, (double)INFINITY, nullptr);
			}
		}
		
		int startID = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalID = gSpatialDatabase->getCellIndexFromLocation(goal);
		Util::Point startCenter = getPointFromGridIndex(startID);
		Util::Point goalCenter = getPointFromGridIndex(goalID);
		for (int i = 0; i < 3; i++) {
			(*nodeMap[startID]).g[i] = 0;
			(*nodeMap[startID]).h[i] = heuristic(startID, goalID, i);
			(*nodeMap[startID]).f[i] = (*nodeMap[startID]).g[i] + (*nodeMap[startID]).h[i];
			OpenList.at(i).insert(startID);
		}
		
		while (OpenList.at(0).size() > 0) {
			
			for (int i = 1; i < 3; i++) {
				int tmpi = getCurrentNode(i, OpenList.at(i), nodeMap);
				int tmp0 = getCurrentNode(0, OpenList.at(0), nodeMap);
				
				if ((*nodeMap[tmpi]).g[i] <= (*nodeMap[tmp0]).g[0]) {
					
					if (tmpi == goalID) {
						std::cout << tmpi << std::endl;
						return reconstruct_path(i, agent_path, tmpi, nodeMap);
					}
					else {
						int currentNode = tmpi;
						std::cout << "1: "<<currentNode << std::endl;
						ClosedList.at(i).insert(currentNode);
						OpenList.at(i).erase(OpenList.at(i).find(currentNode));
						expand(i, currentNode, goalID, OpenList.at(i), ClosedList.at(i), nodeMap);
						
					}
				}
				else {
					
					if (tmp0 == goalID) {
						std::cout << tmp0 << std::endl;
						return reconstruct_path(0, agent_path, tmp0, nodeMap);
					}
					else {
						int currentNode = tmp0;
						std::cout << "2: " << currentNode << std::endl;
						ClosedList.at(0).insert(currentNode);
						OpenList.at(0).erase(OpenList.at(0).find(currentNode));
						expand(0, currentNode, goalID, OpenList.at(0), ClosedList.at(0), nodeMap);
						
					}
				}

			}
			
		}

		return false;
	}

	double AStarPlanner::heuristic(int startIndex, int endIndex, int method) {
		unsigned int startx, startz, endx, endz;
		gSpatialDatabase->getGridCoordinatesFromIndex(startIndex, startx, startz);
		gSpatialDatabase->getGridCoordinatesFromIndex(endIndex, endx, endz);
		if (method == 0) {
			//use Euclidean

			return ((double)sqrt((startx - endx)*(startx - endx) + (startz - endz)*(startz - endz)));

		}//use wieghted Euclidean
		else if (method == 1) {
			return 2 * ((double)sqrt((startx - endx)*(startx - endx) + (startz - endz)*(startz - endz)));
		}
		else if (method == 2) {
			// use weighted Manhattahn
			return  2 * ((abs((double)startx - endx) + abs((double)startz - endz)));
		}

	}

	bool AStarPlanner::reconstruct_path(int method, std::vector<Util::Point>& agent_path, int currentNode, std::map<int, AStarPlannerNode*> nodeMap) {
		std::cout << "reconstructing path" << std::endl;
		AStarPlannerNode* temp = nodeMap[currentNode];
		while ((*temp).parent.at(method)) {
			temp = (*temp).parent.at(method);
			agent_path.insert(agent_path.begin(), (*temp).point);
		}
		//std::cout<<"\nPath length: "<<reverse.size()<<'\n';
		return true;
	}

	int AStarPlanner::getCurrentNode(int method, std::set<int> openset, std::map<int, AStarPlannerNode*> nodeMap) {
		std::set<int>::iterator it;
		double temp = INFINITY;
		//If bigger is true, larger g scores have precedence, else smaller scores have precedence
		bool bigger = true;
		for (std::set<int>::iterator i = openset.begin(); i != openset.end(); ++i) {
			if ((*nodeMap[(*i)]).f[method] < temp) {
				temp = (*nodeMap[(*i)]).f[method];
				it = i;
			}
			else if ((*nodeMap[(*i)]).f[method] == temp) {
				if (bigger) {
					if ((*nodeMap[(*it)]).g[method] < (*nodeMap[(*i)]).g[method]) {
						it = i;
					}
				}
				else {
					if ((*nodeMap[(*it)]).g[method] > (*nodeMap[(*i)]).g[method]) {
						it = i;
					}
				}
			}
		}
		return (*it);
	}

	void AStarPlanner::expand(int method, int currentNode, int goalIndex, std::set<int>& openset, std::set<int>& closedset, std::map<int, AStarPlannerNode*>& nodeMap) {
		//std::cout << "expand" << std::endl;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(currentNode, x, z);
		
		for (int i = MAX(x - 1, 0); i < MIN(x + 2, gSpatialDatabase->getNumCellsX()); i += GRID_STEP) {
			for (int j = MAX(z - 1, 0); j < MIN(z + 2, gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) {
				int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				if (canBeTraversed(neighbor) && closedset.count(neighbor) == 0) {
					double tempg;
					if ((i == x) || (j == z)) {
						tempg = (*nodeMap[currentNode]).g[method] + (DIAGONAL_COST*gSpatialDatabase->getTraversalCost(neighbor));
					}
					else {
						tempg = (*nodeMap[currentNode]).g[method] + gSpatialDatabase->getTraversalCost(neighbor);
					}
					if (tempg < (*nodeMap[neighbor]).g[method]) {
						(*nodeMap[neighbor]).g[method] = tempg;
						(*nodeMap[neighbor]).f[method] = (*nodeMap[neighbor]).g[method] + heuristic(neighbor, goalIndex, method);
						if (openset.count(neighbor) == 1) {
							openset.erase(openset.find(neighbor));
						}
						openset.insert(neighbor);
						(*nodeMap[neighbor]).parent.at(method) = nodeMap[currentNode];
					}
				}
			}
		}
	}

}