/*
 * IDAStar.h
 *
 *  Created on: 2018Äê9ÔÂ12ÈÕ
 *      Author: ygli
 */

#ifndef IDASTAR_H_
#define IDASTAR_H_

#include <stack>
#include "globalVariables.h"

class IdaStar {
public:
	IdaStar(int rows, int cols);
	bool findPath();
	void setStart(int startX, int startY);
	void setGoal(int goalX, int goalY);
	void updateH();
	IdaStarPath* getPath();

	friend void updateData(bool fromMazeToMap);
private:
	double calcH(MazeCell* s);
	double cost(MazeCell* u, MazeCell* s);
	void getNeighbours(MazeCell* u, MazeCell** ptrNeighbours);
	void initialise();

	/**
	 * the path can not be expanded if for all its successors' f-cost exceeds the threshold
	 */
	void expandPath(IdaStarPath* path);
private:
	double m_cutoff;
	double m_nextCutOff; //the minimum among the f-costs that exceeded the threshold
	vector<vector<MazeCell> > m_maze;
	stack<IdaStarPath> m_path;
	int m_rows, m_cols;
	MazeCell* m_pStart;
	MazeCell* m_pGoal;

};

#endif /* IDASTAR_H_ */
