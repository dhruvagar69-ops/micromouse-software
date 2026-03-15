#ifndef MAZESOLVER_CONF_H_
#define MAZESOLVER_CONF_H_

/***************************************
* Parameters related to the maze
***************************************/
//Maze size
#define MAZE_SIZE 8

// Maze goal coordinates
// Arrange the coordinates you want to be the goal
// It doesn't have to be 4 coordinates
#define MAZE_GOAL_LIST {IndexVec(1,0)}

/****************************************
* Parameters related to the search algorithm
****************************************/
// k when calculating the k shortest paths after reaching the goal
#define SEARCH_DEPTH1

//When the search is finished and the final route is calculated, k
#define SEARCH_DEPTH2

//Robot's driving performance used when calculating the cost of the path
//Time taken to navigate a block with a 90-degree turn [s]
#define TURN90_TIME

//Time taken to navigate a block that turns at a 45-degree angle [s]
#define TURN45_TIME

//Acceleration [m/s^2]
#define ACCELERATION

//Maximum speed [m/s]
#define MAX_VELOCITY

//Minimum speed [m/s]
#define MIN_VELOCITY 

//Length of one plot [m]
#define MAZE_1BLOCK_LENGTH 	0.25

#endif /* MAZESOLVER_CONF_H_ */