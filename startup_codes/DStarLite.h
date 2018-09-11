#ifndef __DSTARLITE_H__
#define __DSTARLITE_H__

#include "PriorityQueue.h"

class DStarLite{
public:
	DStarLite(int rows, int cols);
	void setStart(int startX, int startY);
	void setGoal(int goalX, int goalY);
	bool findPath();
	void updateH();
	void updateKey();

	friend void updateData(bool fromMazeToMap);
//private:
	void initialize();

	double* calcKey(MazeCell* s);

	double cost(MazeCell* u, MazeCell* s);
	//h(start, s)
	double calcH(MazeCell* s);

	//find vertex s who has the minimum c(u,s)+g(s)
	double minCPlusG(MazeCell* u, MazeCell** moveTo);
	//if key1 less than key2
	static bool lessThan(double key1[2], double key2[2]);
	static bool equal(double d1, double d2);
	//copy sourceKey to targetKey
	static void copyKey(double* targetKey, double* sourceKey);
	static bool isSame(MazeCell* v1, MazeCell* v2);
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
	int m_rows, m_cols;
	double m_km;
	vector<vector<MazeCell> > m_maze;
	MazeCell* m_pStart;
	MazeCell* m_pGoal;
	MazeCell* m_pLast;
	PriorityQueue m_U;
};


#endif //__DSTARLITE_H__
