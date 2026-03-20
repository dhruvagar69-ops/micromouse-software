#ifndef SHORTESTPATH_H_
#define SHORTESTPATH_H_

#include <list>
#include <vector>

#include "Maze.h"
#include "Operation.h"

typedef std::vector<IndexVec> Path;
/**************************************************************
* ShortestPath
* Algorithm for finding shortest path
* Calculation of the shortest path using a step count map
* Calculation of k shortest paths using Yes's Algorithm
* Estimation of path travel time based on robot travel parameters
**************************************************************/
class ShortestPath {
private:
	Maze *maze;

    //Save the various calculated routes.
	Path shortestDistancePath;
	std::vector< Path > k_shortestDistancePath;
	int shortestTimePath_index;
	OperationList shortestTimePath_operationList;
	float shortestTimePath_cost;
	std::list<IndexVec> needToSearchWallIndex;

    //Used within the k-shortest-path function
	void removeEdge(const IndexVec& start, const IndexVec& end);
	void removeNode(const IndexVec& node);
	bool matchPath(const Path &path1, const Path &path2, int n);

public:
	ShortestPath(Maze &_maze, bool _useDiagonalPath = false)
: maze(&_maze), shortestTimePath_index(-1)
{
		clear();
}

	void clear() {
		shortestDistancePath.clear();
		needToSearchWallIndex.clear();
		shortestTimePath_index=-1;
	}

    // Calculates the shortest path from start to goal and stores it in shortestDistancePath.
    // If goalList is given, calculates the path to the nearest coordinates among those included in goalList.
    // When onlyUseFoundWall=true, generates a path that does not pass through unexplored walls.
	int calcShortestDistancePath(const IndexVec &start, const IndexVec &goal, bool onlyUseFoundWall);
	int calcShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, bool onlyUseFoundWall);
	inline const Path &getShortestDistancePath() const { return shortestDistancePath; }

    // Calculates the k shortest path
    // Behaves the same as calcShortestDistancePath
    // The result is stored in k_shortestDistancePath
	int calcKShortestDistancePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall);
	int calcKShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall);
	inline const std::vector< Path > &getKShortestDistancePath() const { return k_shortestDistancePath; }

    // Calculates the shortest (likely) path in terms of time.
    // Internally, k_shortestDistancePath is executed to calculate the travel time for k paths.
    // The one with the smallest cost (travel time) is set as ShortestTimePath.
    // The index of the shortest path (from k_shortestDistancePath) is stored in shortestTimePath_index.
	int calcShortestTimePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall, bool useDiagonalPath);
	int calcShortestTimePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall, bool useDiagonalPath);
	inline const Path &getShortestTimePath() const { return k_shortestDistancePath[shortestTimePath_index]; }
	inline const OperationList &getShortestTimePathOperation() const { return shortestTimePath_operationList; }
	inline float getShortestTimePathCost() const { return shortestTimePath_cost; }

    //Calculates a list of coordinates on kShortestDistancePath where walls have not yet been explored.
    //These coordinates are the ones that need to be explored next.
    //Execute this after running calcKShortestDistancePath
	void calcNeedToSearchWallIndex();
	inline const std::list<IndexVec> &getNeedToSearchIndex() const { return needToSearchWallIndex; }
};


#endif /* SHORTESTPATH_H_ */