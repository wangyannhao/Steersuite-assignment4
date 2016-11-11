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
				nodeMap[index] = new AStarPlannerNode(getPointFromGridIndex(index), (double)INFINITY, (double)INFINITY, (double)INFINITY, nullptr);
			}
		}
		int startID = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalID = gSpatialDatabase->getCellIndexFromLocation(goal);
		Util::Point startCenter = getPointFromGridIndex(startID);
		Util::Point goalCenter = getPointFromGridIndex(goalID);
		(*nodeMap[startID]).g = 0;
		(*nodeMap[startID]).f = (*nodeMap[startID]).g + HEURISTIC_WEIGHT*heuristic(startID, goalID);

		std::set<int> closedSet;
		std::set<int> openSet;
		openSet.insert(startID);

		while (!openSet.empty()) {
			//Find node in openset with smallest f value
			int currentNode = getCurrentNode(openSet, nodeMap);

			//Add to closedset, remove from openset
			closedSet.insert(currentNode);
			openSet.erase(openSet.find(currentNode));

			//Check if we reached the goal
			if (currentNode == goalID) {
				return reconstruct_path(agent_path, currentNode, nodeMap);
			}

			//Search through neighbors, calculate g,f scores, add to openset
			expand(currentNode, goalID, openSet, closedSet, nodeMap);
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

	void AStarPlanner::expand(int currentNode, int goalIndex, std::set<int>& openset, std::set<int> closedset, std::map<int, AStarPlannerNode*>& nodeMap) {
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(currentNode, x, z);
		for (int i = MAX(x - 1, 0); i<MIN(x + 2, gSpatialDatabase->getNumCellsX()); i += GRID_STEP) {
			for (int j = MAX(z - 1, 0); j<MIN(z + 2, gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) {
				int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				if (canBeTraversed(neighbor) && closedset.count(neighbor) == 0) {
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
						if (openset.count(neighbor) == 1) {
							openset.erase(openset.find(neighbor));
						}
						openset.insert(neighbor);
						(*nodeMap[neighbor]).parent = nodeMap[currentNode];
					}
				}
			}
		}
	}



	//------------------------------------------------------------------------dynamic AStar---------------------------------------------------------------------------
	bool AStarPlanner::dynamicComputePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface  * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//Initialize f and g map scores
		std::map<int, AStarPlannerNode*> nodeMap;
		for (int i = 0; i < gSpatialDatabase->getNumCellsX(); ++i) {
			for (int j = 0; j < gSpatialDatabase->getNumCellsZ(); ++j) {
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				nodeMap[index] = new AStarPlannerNode(getPointFromGridIndex(index), (double)INFINITY, (double)INFINITY, (double)INFINITY, nullptr);
			}
		}
		int startID = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalID = gSpatialDatabase->getCellIndexFromLocation(goal);
		Util::Point startCenter = getPointFromGridIndex(startID);
		Util::Point goalCenter = getPointFromGridIndex(goalID);
		(*nodeMap[startID]).g = (*nodeMap[startID]).rhs = (double)INFINITY;
		(*nodeMap[goalID]).g = (double)INFINITY;
		(*nodeMap[goalID]).rhs = 0;
		//(*nodeMap[startID]).f = (*nodeMap[startID]).g + HEURISTIC_WEIGHT*heuristic(startID, goalID);

		//dynamic list
		std::set<int> dclosedSet;
		std::set<int> dincons;
		BinaryHeap dopenSet;
		nodeKey thisKey;
		thisKey.node = startID;
		thisKey.key = key(startID, startID, nodeMap);
		dopenSet.Insert(thisKey);

		computeorImprovePath(startID, goalID, dopenSet, dclosedSet, dincons, nodeMap);

		return false;
	}
	
	void AStarPlanner::computeorImprovePath(int startID, int goalID, BinaryHeap &dopenSet, std::set<int> &dclosedSet, std::set<int>& dincons, std::map<int, AStarPlannerNode*>& nodeMap) {
		while ((dopenSet.Size()) != 0) {
			//Find node in openset with smallest f value
			//int currentNode = getCurrentNode(openSet, nodeMap);
			nodeKey currentKey = dopenSet.ExtractMin();
			int currentNode = currentKey.node;
			if (compareKey(currentKey.key, key(startID, startID, nodeMap)) || (*nodeMap[startID]).rhs != (*nodeMap[startID]).g) {

				if ((*nodeMap[currentNode]).g > (*nodeMap[currentNode]).rhs) {
					(*nodeMap[currentNode]).g = (*nodeMap[currentNode]).rhs;
					dclosedSet.insert(currentNode);
					int parentInt;
					std::map<int, AStarPlannerNode*>::const_iterator it;
					for (it = nodeMap.begin(); it != nodeMap.end();it++) {
						if (it->second == (*nodeMap[currentNode]).parent) {
							parentInt = it->first;
							break;
						}
					}
					updateState(parentInt, startID, goalID, dopenSet, dclosedSet, dincons, nodeMap);
				}
				else {
					(*nodeMap[currentNode]).g = (double)INFINITY;
					int parentInt;
					std::map<int, AStarPlannerNode*>::const_iterator it;
					for (it = nodeMap.begin(); it != nodeMap.end();it++) {
						if (it->second == (*nodeMap[currentNode]).parent) {
							parentInt = it->first;
							break;
						}
					}
					updateState(parentInt, startID, goalID, dopenSet, dclosedSet, dincons, nodeMap);
					updateState(currentNode, startID, goalID, dopenSet, dclosedSet, dincons, nodeMap);
				}

				/*
				//Add to closedset, remove from openset
				closedSet.insert(currentNode);
				openSet.erase(openSet.find(currentNode));

				//Check if we reached the goal
				if (currentNode == goalID) {
				return reconstruct_path(agent_path, currentNode, nodeMap);
				}

				//Search through neighbors, calculate g,f scores, add to openset
				expand(currentNode, goalID, openSet, closedSet, nodeMap);
				*/
			}
		}
		//std::cout<<"\nIn A*";
	}

	keyValue AStarPlanner::key(int currentNode, int start, std::map<int, AStarPlannerNode*>& nodeMap) {
		keyValue thisKey;
		if ((*nodeMap[currentNode]).g > (*nodeMap[currentNode]).rhs) {
			thisKey.k1 = (*nodeMap[currentNode]).rhs + HEURISTIC_WEIGHT*heuristic(start, currentNode);
			thisKey.k2 = (*nodeMap[currentNode]).rhs;
			return thisKey;
		}
		else {
			thisKey.k1 = (*nodeMap[currentNode]).g + HEURISTIC_WEIGHT*heuristic(start, currentNode);
			thisKey.k2 = (*nodeMap[currentNode]).g;
			return thisKey;
		}
	}

	void AStarPlanner::updateState(int currentNode, int start, int goal, BinaryHeap &dopenset, std::set<int> &dclosedset, std::set<int>& dincons, std::map<int, AStarPlannerNode*>& nodeMap) {
		if (dclosedset.count(currentNode) == 0) {
			(*nodeMap[currentNode]).g = (double)INFINITY;
		}
		if (currentNode != goal) {
			unsigned int x, z;
			gSpatialDatabase->getGridCoordinatesFromIndex(currentNode, x, z);
			double minimum = (double)INFINITY;
			for (int i = MAX(x - 1, 0); i<MIN(x + 2, gSpatialDatabase->getNumCellsX()); i += GRID_STEP) {
				for (int j = MAX(z - 1, 0); j<MIN(z + 2, gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) {
					int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
					if (canBeTraversed(neighbor) && dclosedset.count(neighbor) == 0) {
						double tempg;
						if ((i == x) || (j == z)) {
							tempg = (*nodeMap[currentNode]).g + gSpatialDatabase->getTraversalCost(neighbor);
						}
						else {
							tempg = (*nodeMap[currentNode]).g + (DIAGONAL_COST*gSpatialDatabase->getTraversalCost(neighbor));
						}
						if (tempg < minimum)
							minimum = tempg;
					}
				}
			}
			(*nodeMap[currentNode]).rhs = minimum;
		}
		if (dopenset.Find(currentNode)) {
			dopenset.DeleteAny(currentNode);
		}
		if ((*nodeMap[currentNode]).g != (*nodeMap[currentNode]).rhs) {
			if (dclosedset.count(currentNode) == 0) {
				nodeKey newKey;
				newKey.node = currentNode;
				newKey.key = key(currentNode, start, nodeMap);
				dopenset.Insert(newKey);
			}
			else {
				dincons.insert(currentNode);
			}
		}
	}




	//--------------------------------------------------------------------------binary minheap---------------------------------------------------------
	/*
	* Return Heap Size
	*/
	int BinaryHeap::Size()
	{
		return heap.size();
	}

	/*
	* Insert Element into a Heap
	*/
	void BinaryHeap::Insert(nodeKey element)
	{
		heap.push_back(element);
		heapifyup(heap.size() - 1);
	}
	/*
	* Delete Minimum Element
	*/
	void BinaryHeap::DeleteMin()
	{
		if (heap.size() == 0)
		{
			//do nothing for now
			std::cout << "The heap is already empty, you can not delete.";
			return;
		}
		heap[0] = heap.at(heap.size() - 1);
		heap.pop_back();
		heapifydown(0);
	}

	/*
	* Extract Minimum Element
	*/
	nodeKey BinaryHeap::ExtractMin()
	{
		if (heap.size() == 0)
		{
			//return -1;
			std::cout << "Heap is already empty, can not extract the min one.";
		}
		else {
			return heap.front();
			DeleteMin();
		}
			
	}

	/*
	* Return Left Child
	*/
	int BinaryHeap::left(int parent)
	{
		int l = 2 * parent + 1;
		if (l < heap.size())
			return l;
		else
			return -1;
	}

	/*
	* Return Right Child
	*/
	int BinaryHeap::right(int parent)
	{
		int r = 2 * parent + 2;
		if (r < heap.size())
			return r;
		else
			return -1;
	}

	/*
	* Return Parent
	*/
	int BinaryHeap::parent(int child)
	{
		int p = (child - 1) / 2;
		if (child == 0)
			return -1;
		else
			return p;
	}

	/*
	* Heapify- Maintain Heap Structure bottom up
	*/
	void BinaryHeap::heapifyup(int in)
	{
		if (in >= 0 && parent(in) >= 0 && compareKey(heap[in].key, heap[parent(in)].key))
		{
			nodeKey temp = heap[in];
			heap[in] = heap[parent(in)];
			heap[parent(in)] = temp;
			heapifyup(parent(in));
		}
	}

	/*
	* Heapify- Maintain Heap Structure top down
	*/
	void BinaryHeap::heapifydown(int in)
	{

		int child = left(in);
		int child1 = right(in);
		if (child >= 0 && child1 >= 0 && compareKey(heap[child1].key, heap[child].key))
		{
			child = child1;
		}
		if (child > 0)
		{
			nodeKey temp = heap[in];
			heap[in] = heap[child];
			heap[child] = temp;
			heapifydown(child);
		}
	}

	bool BinaryHeap::Find(int index) {
		bool exist = false;
		for (int i = 0; i < heap.size(); i++) {
			if (heap[i].node == index) {
				exist = true;
				break;
			}
		}
		return exist;
	}

	void BinaryHeap::DeleteAny(int value) {
		int index;
		for (int i = 0; i < heap.size(); i++) {
			if (heap[i].node == value) {
				index = i;
				break;
			}
		}
		if (index == heap.size() - 1) {
			heap.pop_back();
		}
		else {
			nodeKey temp = heap[heap.size() - 1];
			heap.pop_back();
			heap[index] = temp;
			heapifyup(index);
			heapifydown(index);
		}
	}

	// return true if firstkey < secondkey, else return false
	bool compareKey(keyValue firstKey, keyValue seondkey) {
		if (firstKey.k1 < seondkey.k1) {
			return true;
		}
		else if (firstKey.k1 == seondkey.k1 && firstKey.k2 < seondkey.k2) {
			return true;
		}
		else {
			return false;
		}
	}
}