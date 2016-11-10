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
		(*nodeMap[startID]).g = 0;
		(*nodeMap[startID]).f = (*nodeMap[startID]).g + w*heuristic(startID, goalID);

		(*nodeMap[startID]).g = INFINITE;
		(*nodeMap[startID]).f = (*nodeMap[startID]).g;

		std::set<int> closedSet;
		std::set<int> openSet;
		std::set<int> INCONS;
		openSet.insert(startID);
		expand(startID, goalID, openSet, closedSet, INCONS, nodeMap);

		if (w < (*nodeMap[(goalID)]).g / getCurrentNode(and (openSet, INCONS), nodeMap)) {
			_w = w;
		}
		else {
			_w = (*nodeMap[(goalID)]).g / getCurrentNode(and (openSet, INCONS), nodeMap);
		}
		while (_w > 1) {
			//decrease w;
			w = w*0.8;

			openSet = and(openSet, INCONS);
			int currentNode;
			while ((*nodeMap[(goalID)]).g > getCurrentNode(openSet, nodeMap)) {
				//Find node in openset with smallest f value
				currentNode = getCurrentNode(openSet, nodeMap);

				//Add to closedset, remove from openset
				closedSet.insert(currentNode);
				openSet.erase(openSet.find(currentNode));

				//Check if we reached the goal
				//if (currentNode == goalID) {
				//	return reconstruct_path(agent_path, currentNode, nodeMap);
				//}

				//Search through neighbors, calculate g,f scores, add to openset
				expand(currentNode, goalID, openSet, closedSet, INCONS, nodeMap);
			}
			if (w < (*nodeMap[(goalID)]).g / getCurrentNode(and (openSet, INCONS), nodeMap)) {
				_w = w;
			}
			else {
				_w = (*nodeMap[(goalID)]).g / getCurrentNode(and (openSet, INCONS), nodeMap);
			}
			reconstruct_path(agent_path, currentNode, nodeMap);
		}
		if (_w <= 1 && agent_path.size()!=0) {
			return true;
		}
		//std::cout<<"\nIn A*";
		return false;
	}

	double AStarPlanner::heuristic(int startIndex, int endIndex) {
		//If method is true, use Euclidean, else use Manhattan
		bool method = true;
		unsigned int startx, startz, endx, endz;
		gSpatialDatabase->getGridCoordinatesFromIndex(startIndex, startx, startz);
		gSpatialDatabase->getGridCoordinatesFromIndex(endIndex, endx, endz);
		if (method) {
			return ((double)sqrt((startx - endx)*(startx - endx) + (startz - endz)*(startz - endz)));
		}
		else {
			return ((abs((double)startx - endx) + abs((double)startz - endz)));
		}
	}

	bool AStarPlanner::reconstruct_path(std::vector<Util::Point>& agent_path, int currentNode, std::map<int, AStarPlannerNode*> nodeMap) {
		AStarPlannerNode* temp = nodeMap[currentNode];
		while ((*temp).parent) {
			temp = (*temp).parent;
			agent_path.insert(agent_path.begin(), (*temp).point);
		}
		//std::cout<<"\nPath length: "<<reverse.size()<<'\n';
		return true;
	}

	int AStarPlanner::getCurrentNode(std::set<int> openset, std::map<int, AStarPlannerNode*> nodeMap) {
		std::set<int>::iterator it;
		double temp = INFINITY;
		//If bigger is true, larger g scores have precedence, else smaller scores have precedence
		bool bigger = true;
		for (std::set<int>::iterator i = openset.begin(); i != openset.end(); ++i) {
			if ((*nodeMap[(*i)]).f < temp) {
				temp = (*nodeMap[(*i)]).f;
				it = i;
			}
			else if ((*nodeMap[(*i)]).f == temp) {
				if (bigger) {
					if ((*nodeMap[(*it)]).g < (*nodeMap[(*i)]).g) {
						it = i;
					}
				}
				else {
					if ((*nodeMap[(*it)]).g >(*nodeMap[(*i)]).g) {
						it = i;
					}
				}
			}
		}
		return (*it);
	}

	std::set<int> AStarPlanner::and (std::set<int> a, std::set<int>b) {
		for (int i = 0; i < b.size(); i++)
		{
			int tmp = b.end();
			a.insert(tmp);
			b.erase(b.end());
		}
		
	}

	void AStarPlanner::expand(int currentNode, int goalIndex, std::set<int>& openset, std::set<int> closedset,std::set<int> INCONS, std::map<int, AStarPlannerNode*>& nodeMap) {
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(currentNode, x, z);
		for (int i = MAX(x - 1, 0); i<MIN(x + 2, gSpatialDatabase->getNumCellsX()); i += GRID_STEP) {
			for (int j = MAX(z - 1, 0); j<MIN(z + 2, gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) {
				int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				if (canBeTraversed(neighbor) ) {
					double tempg;
					if ((i == x) || (j == z)) {
						tempg = (*nodeMap[currentNode]).g + (DIAGONAL_COST*gSpatialDatabase->getTraversalCost(neighbor));
					}
					else {
						tempg = (*nodeMap[currentNode]).g + gSpatialDatabase->getTraversalCost(neighbor);
					}
					if (tempg < (*nodeMap[neighbor]).g) {
						(*nodeMap[neighbor]).g = tempg;
						(*nodeMap[neighbor]).f = (*nodeMap[neighbor]).g + HEURISTIC_WEIGHT*heuristic(neighbor, goalIndex);
						//if neighbor is in the openset
						if (openset.count(neighbor) == 1) {
							openset.erase(openset.find(neighbor));
						}
						//if neighbor is not in the closedset
						if (closedset.count(neighbor) == 0) {
							openset.insert(neighbor);
							(*nodeMap[neighbor]).parent = nodeMap[currentNode];
						}
						//neighbor is in the closeset
						else {
							INCONS.insert(neighbor);
						}
					}
				}
				
			}
		}
	}

}