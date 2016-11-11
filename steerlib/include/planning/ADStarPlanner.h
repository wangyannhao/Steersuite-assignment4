﻿
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{
	struct keyValue
	{
		double k1;
		double k2;
	};
	
	struct nodeKey
	{
		int node;
		keyValue key;
	};

	class BinaryHeap
	{
	private:
		std::vector <nodeKey> heap;
		int left(int parent);
		int right(int parent);
		int parent(int child);
		void heapifyup(int index);
		void heapifydown(int index);
	public:
		BinaryHeap()
		{}
		void Insert(nodeKey element);
		void DeleteMin();
		nodeKey ExtractMin();
		int Size();
		bool Find(int index);
		void DeleteAny(int index);
	};

	bool compareKey(keyValue k1, keyValue k2);
	/*
	@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
	@attributes
	f : the f value of the node
	g : the cost from the start, for the node
	point : the point in (x,0,z) space that corresponds to the current node
	parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
	@operators
	The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation
	*/
	class STEERLIB_API AStarPlannerNode {
	public:
		double f;
		double g;
		double rhs;
		Util::Point point;
		AStarPlannerNode* parent;
		AStarPlannerNode(Util::Point _point, double _g, double _f, double _rhs, AStarPlannerNode* _parent)
		{
			f = _f;
			point = _point;
			g = _g;
			rhs = _rhs;
			parent = _parent;
		}
		bool operator<(AStarPlannerNode other) const
		{
			return this->f < other.f;
		}
		bool operator>(AStarPlannerNode other) const
		{
			return this->f > other.f;
		}
		bool operator==(AStarPlannerNode other) const
		{
			return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		}

	};

	class STEERLIB_API AStarPlanner {
	public:
		AStarPlanner();
		~AStarPlanner();
		// NOTE: There are four indices that need to be disambiguated
		// -- Util::Points in 3D space(with Y=0)
		// -- (double X, double Z) Points with the X and Z coordinates of the actual points
		// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
		// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
		// When navigating the space or the Grid, do not mix the above up

		/*
		@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
		The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
		and checks cells in bounding box area
		[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
		[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
		This function also contains the griddatabase call that gets traversal costs.
		*/
		bool canBeTraversed(int id);
		/*
		@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
		*/
		Util::Point getPointFromGridIndex(int id);

		/*
		@function computePath
		DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
		This function executes an A* query
		@parameters
		agent_path : The solution path that is populated by the A* search
		start : The start point
		goal : The goal point
		_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
		append_to_path : An optional argument to append to agent_path instead of overwriting it.
		*/

		bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface* _gSpatialDatabase, bool append_to_path = false);
	private:
		SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
		double heuristic(int startIndex, int endIndex);
		bool reconstruct_path(std::vector<Util::Point>& agent_path, int currentNode, std::map<int, AStarPlannerNode*> nodeMap);
		int getCurrentNode(std::set<int> openset, std::map<int, AStarPlannerNode*> nodeMap);
		void expand(int currentNode, int goalIndex, std::set<int>& openset, std::set<int> closedset, std::map<int, AStarPlannerNode*>& nodeMap);

		//for dynamic AStar
		bool dynamicComputePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface  * _gSpatialDatabase, bool append_to_path);
		keyValue key(int currentnode, int start, std::map<int, AStarPlannerNode*>& nodeMap);
		void updateState(int currentNode, int start, int goal, BinaryHeap &dopenset, std::set<int> &dclosedset, std::set<int>& dincons, std::map<int, AStarPlannerNode*>& nodeMap);
		void computeorImprovePath(int startID, int goalID, BinaryHeap &dopenset, std::set<int> &dclosedset, std::set<int>& dincons, std::map<int, AStarPlannerNode*>& nodeMap);
	};

}


#endif