#ifndef MAZESOLVER_CONF_H_
#define MAZESOLVER_CONF_H_

/***************************************
* Parameters related to the maze
***************************************/
//Maze size
#define MAZE_SIZE 8

// Maze goal coordinates
// list the coordinates you want to be the goal
// Does not have to be exactly 4
#define MAZE_GOAL_LIST {IndexVec(1,0)}

/****************************************
* Parameters related to the search algorithm
****************************************/
// k when calculating the k shortest paths after reaching the goal
#define SEARCH_DEPTH1 1

//k used when the search ends and the final route is calculated
#define SEARCH_DEPTH2 20

//Robot's driving performance used when calculating the cost of the path
//Time taken to navigate a block with a 90-degree turn [s]
#define TURN90_TIME 0.6

//Time taken to navigate a block that turns at a 45-degree angle [s]
#define TURN45_TIME 0.4

//Acceleration [m/s^2]
#define ACCELERATION 6.0

//Maximum speed [m/s]
#define MAX_VELOCITY 1.0

//Minimum speed [m/s]
#define MIN_VELOCITY 0.2

//Length of one plot [m]
#define MAZE_1BLOCK_LENGTH 	0.25

#endif /* MAZESOLVER_CONF_H_ */