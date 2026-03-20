#ifndef OPERATION_H_
#define OPERATION_H_

#include <vector>
#include <cstddef>
#include "Maze.h"
#include "MazeSolver_conf.h"


typedef std::vector<IndexVec> Path;

/**************************************************************
 * Operation
 *    Describes the actions(movements) the robot should perform.
 *    Used to describe a movement path.
 *    “op” refers to the type of movement; it means “execute op n times”.
 **************************************************************/
struct Operation {
	typedef enum {
		FORWARD,
		FORWARD_DIAG,
		TURN_RIGHT90,
		TURN_RIGHT45,
		TURN_LEFT90,
		TURN_LEFT45,
		STOP,
	} OperationType;

	OperationType op;
	uint8_t n;
	Operation(OperationType _op = STOP, uint8_t _n = 1) : op(_op), n(_n) {}
};

/**************************************************************
 * Operation List
 *    Maintaining a sequence of operations from start to finish.
 *    If you enter the constructor's path, it will automatically convert it(the constructor will automatically convert the path you enter).
 **************************************************************/
class OperationList {
private:
	std::vector<Operation> opList;

public:
	OperationList() { }
	//It automatically converts and stores the path you enter.
	OperationList(const Path &path, bool useDiagonalPath) { loadFromPath(path, useDiagonalPath); }
	OperationList(const OperationList &obj) { opList = obj.opList; }

	const OperationList &operator=(const OperationList &rhs)
	{
		opList = rhs.opList;
		return (*this);
	}

	//An interface similar to std::vector
	inline std::vector<Operation>::const_iterator begin() const {return opList.begin(); }
	inline std::vector<Operation>::const_iterator end() const { return opList.end(); }
	inline size_t size() const { return opList.size(); }
	inline void push_back(const Operation& op) { opList.push_back(op); }
	inline void pop_back() { opList.pop_back(); }
	const Operation &operator[](size_t i) const { return opList[i]; }

	//Calculate and return the total cost (in time) of all operations in opList.
    //The time required is calculated by simulating the machine's movement.
	float eval() const;

	//Load path
    //Convert to an operation and save it to the member's OpList.
    //Setting useDiagonalPath=true enables conversion with diagonal movement enabled.
	void loadFromPath(const Path& path, bool useDiagonalPath);

	void print();
};


#endif /* OPERATION_H_ */