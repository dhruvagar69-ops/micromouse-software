#ifndef MAZE_H_
#define MAZE_H_

#include <cstdint>
#include <cstddef>
#include "MazeSolver_conf.h"

/**************************************************************
 * Direction
 *used to represent information about walls in a cell, the direction of movement, and whether the walls have been explored or not
 **************************************************************/
union __attribute__ ((__packed__)) Direction {
public:
	uint8_t byte;
	struct __attribute__ ((__packed__)) {
		uint8_t North:1; 		//bit0 LSB
		uint8_t East:1; 		//bit1
		uint8_t South:1; 		//bit2
		uint8_t West:1; 		//bit3
		uint8_t DoneNorth:1; 	//bit4
		uint8_t DoneEast:1; 	//bit5
		uint8_t DoneSouth:1; 	//bit6
		uint8_t DoneWest:1; 	//bit7 MSB
	} bits;
public:
	Direction(uint8_t value=0) : byte(value) {}

	//all calculations are performed after casting to uint8_t, so that the Done* bits are not affected by the operations
	inline operator uint8_t() const { return byte; }
	inline uint8_t operator|(uint8_t value) const { return byte | value; }
	inline uint8_t operator&(uint8_t value) const { return byte & value; }
	inline uint8_t operator=(uint8_t value) { return byte = value; }
	inline uint8_t operator|=(uint8_t value) { return byte |= value; }
	inline uint8_t operator&=(uint8_t value) { return byte &= value; }
	inline uint8_t operator=(Direction &obj) { return byte = obj.byte; }

	//returns a uint8_t value of 0x00 or 0x01 indicating whether the index-th bit is set or not
	inline uint8_t operator[](uint8_t index) const {return (byte & (0x01<<index)) ? 1:0; }

	//have all walls been explored?
	inline bool isDoneAll() const { return (byte | 0x0f) == 0xff; }

	//count the number of walls
	int nWall() const {
		int cnt = 0;
		if (bits.North) cnt++;
		if (bits.East) cnt++;
		if (bits.South) cnt++;
		if (bits.West) cnt++;
		return cnt;
	}

	//count the walls taht have been explored
	int nDoneWall() const {
		int cnt = 0;
		if (bits.DoneNorth) cnt++;
		if (bits.DoneEast) cnt++;
		if (bits.DoneSouth) cnt++;
		if (bits.DoneWest) cnt++;
		return cnt;
	}
};

extern const uint8_t NORTH;
extern const uint8_t EAST;
extern const uint8_t SOUTH;
extern const uint8_t WEST;
extern const uint8_t DONE_NORTH;
extern const uint8_t DONE_EAST;
extern const uint8_t DONE_SOUTH;
extern const uint8_t DONE_WEST;


/**************************************************************
 * IndexVec
 * used to represent the position of a cell in the maze, and to perform vector operations such as addition and subtraction
 *	it has x and y componenets in int8_t, and can be used asa vector for +/- operations and assignmet.
 **************************************************************/
struct __attribute__ ((__packed__)) IndexVec {
	int8_t x;
	int8_t y;

	
	IndexVec(int8_t _x=0, int8_t _y=0) : x(_x), y(_y) {}
	IndexVec(const IndexVec &obj) : x(obj.x), y(obj.y) {}

	// operationns as Vectors
	inline IndexVec operator+(const IndexVec &obj) const { return IndexVec(x+obj.x, y+obj.y); }
	inline IndexVec operator-(const IndexVec &obj) const { return IndexVec(x-obj.x, y-obj.y); }
	inline void operator+=(const IndexVec &obj) { x+=obj.x; y+=obj.y; }
	inline void operator-=(const IndexVec &obj) { x-=obj.x; y-=obj.y; }
	inline const IndexVec& operator=(const IndexVec &obj) { x=obj.x; y=obj.y; return *this; }
	inline bool operator==(const IndexVec &obj) const { return x == obj.x && y == obj.y; }
	inline bool operator!=(const IndexVec &obj) const { return x != obj.x || y != obj.y; }

	//Whether the sum of myselt and the object will ft=it within the maze coordinate range.
	inline bool canSum(const IndexVec &obj) const
	{
		const int8_t res_x = x + obj.x;
		if (res_x<0 || MAZE_SIZE<=res_x) return false;
		const int8_t res_y = y + obj.y;
		if (res_y<0 || MAZE_SIZE<=res_y) return false;
		return true;
	}
	inline bool canSub(const IndexVec &obj) const
	{
		const int8_t res_x = x - obj.x;
		if (res_x<0 || MAZE_SIZE<=res_x) return false;
		const int8_t res_y = y - obj.y;
		if (res_y<0 || MAZE_SIZE<=res_y) return false;
		return true;
	}

	//L1 norm
	inline uint8_t norm() const
	{
		const int8_t x_abs = x>0?x:-x;
		const int8_t y_abs = y>0?y:-y;
		return x_abs + y_abs;
	}

	inline bool isDiag() const
	{
		const int8_t x_abs = x>0?x:-x;
		const int8_t y_abs = y>0?y:-y;
		return x_abs == 1 && y_abs == 1;
	}

	inline bool isCorner(){ return x == MAZE_SIZE-1 || x == 0 || y == MAZE_SIZE-1 || y == 0; }

	//Constants representing the four cardinal directions as vectors
	//vectors representing each directions
	static const IndexVec vecNorth;
	static const IndexVec vecEast;
	static const IndexVec vecSouth;
	static const IndexVec vecWest;
	//[0]: North [1]: East [2]: South [3]: West
	static const IndexVec vecDir[4];
};

/**************************************************************
 * Maze
 *	Keeps wall information and step count map
*	Wall information will be updated using Maze's updateWall.
**************************************************************/
class Maze {
private:
	Direction wall[MAZE_SIZE][MAZE_SIZE];
	uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];

	// To avoid unnecessary calculations, remember the information from the previous step count map calculation.
	// If the situation is the same as last time, the calculation result will not change, so do not execute.
	bool dirty;
	bool lastOnlyUseFoundWall;
	IndexVec lastStepMapDist;

public:
	Maze() : dirty(true), lastOnlyUseFoundWall(true) { clear(); }
	Maze(const Maze &obj) : dirty(true), lastOnlyUseFoundWall(true)
	{
		for (int i=0;i<MAZE_SIZE;i++) {
			for (int j=0;j<MAZE_SIZE;j++) {
				wall[i][j] = obj.wall[i][j];
				stepMap[i][j] = obj.stepMap[i][j];
			}
		}
	}

	const Maze& operator=(const Maze &obj)
	{
		for (int i=0;i<MAZE_SIZE;i++) {
			for (int j=0;j<MAZE_SIZE;j++) {
				wall[i][j] = obj.wall[i][j];
				stepMap[i][j] = obj.stepMap[i][j];
			}
		}
		return *this;
	}

	// Both wall and stepMap will all become 0.
	void clear();

	// Load a maze from a file
	bool loadFromFile(const char *_filename);

	//Load from array
	//The order of the data in the loaded file and array is the same as when you actually see the maze.
	//Note that Maze.wall is upside down.
	//file[i][j] = ascii[i][j] = wall[MAZE_SIZE-1-i][j]
	void loadFromArray(const char asciiData[MAZE_SIZE+1][MAZE_SIZE+1]);
	//Displays a maze in a formatted way on the console.

	//If you pass an array of numbers as an argument, those numbers will be displayed in each section.
	void printWall(const uint8_t value[MAZE_SIZE][MAZE_SIZE] = nullptr) const;
	// If you pass a boolean array as an argument, * will be displayed in the true section.
	void printWall(const bool value[MAZE_SIZE][MAZE_SIZE]) const;
	// Display step map
	void printStepMap() const;


	// When new wall information is discovered, this is read and the wall information is updated.
	// Not only the information for one coordinate, but also the information for the four neighboring walls is updated to ensure consistency.
	// cur: coordinate newState: wall information
	// forceSetDone = true (default)
	// newState's upper 4 bits are set to 1111 and it is incorporated into wall.
	// This means that all walls at that coordinate have been updated as explored.
	// forceSetDone = false
	// newState is incorporated into wall as is.
	void updateWall(const IndexVec &cur, const Direction &newState, bool forceSetDone = true);

	// Update step map
	// Call this when the step map is needed to update the step map before referencing it.
	// Calculate the step map starting from 0 for the coordinates in dist.
	// If onlyUseFoundWall=true, calculate the step map assuming that unexplored walls cannot be passed through.
	void updateStepMap(const IndexVec &dist, bool onlyUseFoundWall = false);

	//Get wall information at the specified coordinates.
	inline const Direction &getWall(const IndexVec &index) const { return wall[index.y][index.x]; }
	inline const Direction &getWall(int8_t x, int8_t y) const { return wall[y][x]; }

	//Get a step count map for the specified coordinates.
	inline const uint8_t &getStepMap(const IndexVec &index) const { return stepMap[index.y][index.x]; }
	inline const uint8_t &getStepMap(int8_t x, int8_t y) const { return stepMap[y][x]; }

};


#endif /* MAZE_H_ */
