#ifndef AGENT_H_
#define AGENT_H_

#include <list>
#include <vector>

#include "Maze.h"
#include "ShortestPath.h"
#include "Operation.h"

/**************************************************************
 * Agent
 *    Provides movement instructions to the robot during exploration.
 *    Holds the highest level of authority in exploration and shortest path calculation.
 *    After advancing one section, call the update function and input the current coordinates and wall information.
 *    When you run update, the next direction to proceed will be calculated.
 *    Maze information is stored externally but is updated via `Agent::update`
 ***************** *********************************************/
class Agent {
public:
	typedef enum {
		IDLE, 					//Not yet executed.
		SEARCHING_NOT_GOAL, 	//Exploring, but have not reached the goal yet.
		SEARCHING_REACHED_GOAL, //Exploring, have reached the goal once, but are still exploring.
		BACK_TO_START, 			//Returning to the start.
		FINISHED 				//Arrived at the start and are ready to calculate the shortest path.
	} State;

private:
	Maze* maze;
	State state;

	//Current target coordinates
	IndexVec dist;

	//List of Target Coordinates
	std::list<IndexVec> distIndexList;

	//Next, the direction the robot should move in (absolute coordinates)
	Direction nextDir;

	//The program that calculates the shortest path.
	ShortestPath path;

	//Shortest route to the goal
    //For now, use this only when heading to the starting point
	Path toDistinationPath;
	//Which of the paths above?
	size_t toDistinationPath_cnt;

	//The Adachi method calculates the next direction you should move.
	Direction calcNextDirection(const IndexVec &cur, const IndexVec &dist);

public:
	Agent(Maze &_maze) :
			maze(&_maze), state(Agent::IDLE), path(_maze) {
		reset();
	}

	// Set the state to IDLE and clear all path-related data
	void reset();

	// Update status
    // cur: Current coordinates
    // cur_wall: Wall information at the current coordinates (the Done bit is ignored)
	void update(const IndexVec &cur, const Direction &cur_wall);

	// Returns the current state
    // After calling `update", always read this to check the status.
	inline const State &getState() const {
		return state;
	}

	//Returns the direction the robot should move next
    //If 0 is returned, it means the task is complete (the robot should likely stop)
    //Sometimes the result may be the direction opposite to the current orientation by 180 degrees
    //In that case, the robot will likely need to stop briefly and reverse direction
	inline const Direction &getNextDirection() const {
		return nextDir;
	}

	//Force the algorithm to reach the goal
    //this is used if the search is taking too long (e.g., call this after 2 minutes)
	void forceGotoStart() {
		dist = IndexVec(0, 0);
		state = Agent::BACK_TO_START;
	}

	//get current target locations
	inline const IndexVec& getDist() const {
		return dist;
	}
	inline const std::list<IndexVec> &getDistList() const {
		return distIndexList;
	}

	//Finding the current k-shortest path
	inline const std::vector<Path> &getKShortestPath() const {
		return path.getKShortestDistancePath();
	}

	// Calculate the final route to run.
    // Execute when the agent's state is FINISHED
	void calcRunSequence(bool useDiagonalPath);
	inline const Path &getShortestPath() const {
		return path.getShortestTimePath();
	}
	inline const OperationList &getRunSequence() const {
		return path.getShortestTimePathOperation();
	}

	// Resume from where you left off
    // Pass the Agent you want to resume and the maze state
	void resumeAt(State resumeState, Maze &_maze);
};

#endif /* AGENT_H_ */