#ifndef __LPASTAR_H__
#define __LPASTAR_H__

#include <vector> 
#include "globalVariables.h"

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class LpaStar{

public:
    LpaStar(int rows, int cols);
	void setStart(int startX, int startY);
	void setGoal(int goalX, int goalY);
	bool findPath();
    void updateHValues();
    void updateAllKeyValues();

    double minValue(double g_, double rhs_);
    int maxValue(int v1, int v2);
	void calcKey(int x, int y);
    void calcKey(LpaStarCell *cell);
    double calc_H(int x, int y);

    friend void updateData(bool fromMazeToMap);
private:
    void initialise();

private:
    vector<vector<MazeCell> > m_maze;
    int m_rows, m_cols;
	MazeCell* m_pStart;
	MazeCell* m_pGoal;
	MazeCell* m_pLast;
	PriorityQueue m_U;
};

#endif
