#ifndef __LPASTAR_H__
#define __LPASTAR_H__

#include <vector> 
#include "PriorityQueue.h"

class LpaStar{
public:
    LpaStar(int rows, int cols);

	void setStart(int startX, int startY);

	void setGoal(int goalX, int goalY);

	bool findPath();

    void updateH();

    void updateKey();

    friend void updateData(bool fromMazeToMap);
private:
    void initialise();
	double* calcKey(MazeCell* s);
	double calcH(MazeCell* s);
	double cost(MazeCell* u, MazeCell* s);
	//find vertex s who has the minimum c(u,s)+g(s)
	double minCPlusG(MazeCell* u);
	/*collect all predecessor of u and store them in ptrPred
	 * Note: 0 and rows-1 rows, 0 and cols-1 columns are all borders
	 *  1 | 2 | 3
	 * -----------
	 *  4 | U | 5
	 * -----------
	 *  6 | 7 | 8
	 */
	void getNeighbours(MazeCell* u, MazeCell** ptrPred);

	void updateVertex(MazeCell* u);

	void computeShortestPath();
private:
    vector<vector<MazeCell> > m_maze;
    int m_rows, m_cols;
	MazeCell* m_pStart;
	MazeCell* m_pGoal;
	MazeCell* m_pLast;
	PriorityQueue m_U;
};

#endif
