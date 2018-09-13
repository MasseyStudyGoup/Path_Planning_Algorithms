#include "DStarLite.h"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

DStarLite::DStarLite(int rows, int cols) {
	m_rows = rows;
	m_cols = cols;
	m_maze.resize(rows);
	for (int i = 0; i < rows; i++) {
		m_maze[i].resize(cols);
	}

	m_km = 0;

	m_pStart = nullptr;
	m_pGoal = nullptr;
	m_pLast = nullptr;
}

void DStarLite::setStart(int row, int col){
	m_pStart = &m_maze[row][col];
}

void DStarLite::setGoal(int row, int col){
	m_pGoal = &m_maze[row][col];
}

void DStarLite::initialize() {
	m_U.empty();
	m_km = 0;
	for (int i = 1; i < m_rows-1; i++) {
		for (int j = 1; j < m_cols-1; j++) {
			m_maze[i][j].g = INF;
			m_maze[i][j].rhs = INF;
		}
	}

	m_pGoal->rhs = 0;
	calcKey(m_pGoal);
	m_U.insert(m_pGoal);
}

double DStarLite::calcH(MazeCell* s) {
	int diffY = abs(m_pStart->row - s->row);
	int diffX = abs(m_pStart->col - s->col);
	if (HEURISTIC == MANHATTAN) {
		s->h = fmax(diffY, diffX);
	} else { // EUCLIDEAN no need to calculate square root
		s->h = diffY * diffY + diffX * diffX;
	}

	return s->h;
}

void DStarLite::updateH(){
	for (int i=0; i<m_rows; i++){
		for (int j=0; j<m_cols; j++)
			calcH(&m_maze[i][j]);
	}
}

double* DStarLite::calcKey(MazeCell* s) {
	s->key[1] = fmin(s->g, s->rhs);
	s->key[0] = fmin(s->g, s->rhs + calcH(s) + m_km);
	return s->key;
}

void DStarLite::updateKey(){
	for (int i=0; i<m_rows; i++){
		for (int j=0; j<m_cols; j++){
			calcKey(&m_maze[i][j]);
		}
	}
}

bool DStarLite::lessThan(double key1[2], double key2[2]) {
	if (equal(key1[0], key2[0]))
		return key1[1] < key2[1];

	return key1[0] < key2[0];
}

bool DStarLite::equal(double d1, double d2) {
	return (abs(d1 - d2) < 0.00001);
}

void DStarLite::copyKey(double* targetKey, double* sourceKey) {
	targetKey[0] = sourceKey[0];
	targetKey[1] = sourceKey[1];
}

bool DStarLite::isSame(MazeCell* v1, MazeCell* v2) {
	if (v1 == nullptr || v2 == nullptr)
		return false;

	return v1->equals(v2);
}

void DStarLite::getNeighbours(MazeCell* u, MazeCell** ptrPred) {
	for (int i = 0; i < DIRECTIONS; i++)
		ptrPred[i] = nullptr;

	int left = 1; // 0 is border
	int right = m_cols - 2; //m_cols-1 is border
	int top = 1;
	int bottom = m_rows - 2;

	for (int i = 0; i < DIRECTIONS; i++) {
		int x = u->col + neighbours[i].x;
		int y = u->row + neighbours[i].y;
		if (x >= left && x <= right && y >= top && y <= bottom
				&& m_maze[y][x].type != T_BLOCKED)
			ptrPred[i] = &m_maze[y][x];
	}
}

double DStarLite::minCPlusG(MazeCell* u, MazeCell** moveTo) {
	MazeCell* succ[DIRECTIONS];
	getNeighbours(u, succ);
	double cg = INF;
	for (int i = 0; i < DIRECTIONS; i++) {
		MazeCell* s = succ[i];
		if (s == nullptr)
			continue;
		if (u->linkCost[i] + s->g < cg)
		{
			cg = u->linkCost[i] + s->g;
			*moveTo = s;
		}
	}

	return cg;
}

void DStarLite::updateVertex(MazeCell* u) {
	if (u == nullptr)
		return;

	if (!isSame(u, m_pGoal)) {
		MazeCell* result = nullptr;
		u->rhs = minCPlusG(u, &result);
	}

	m_U.remove(u->row, u->col);

	if (!equal(u->g, u->rhs)) {
		calcKey(u);
		m_U.insert(u);
	}
}

void DStarLite::computeShortestPath() {
	double key_old[2];
	MazeCell* Pred[DIRECTIONS];

	while (m_U.size() > 0
			&& (lessThan(m_U.top()->key, calcKey(m_pStart))	|| !equal(m_pStart->rhs, m_pStart->g))) {
		copyKey(key_old, m_U.top()->key);
		MazeCell* u = m_U.pop();
		if (lessThan(key_old, calcKey(u))) {
			m_U.insert(u);
		} else if (u->g > u->rhs) {
			u->g = u->rhs;
			getNeighbours(u, Pred);
			for (int i=0; i < DIRECTIONS; i++) {
				updateVertex(Pred[i]);
			}
		} else {
			u->g = INF;
			getNeighbours(u, Pred);
			for (int i=0; i < DIRECTIONS; i++) {
				updateVertex(Pred[i]);
			}
			updateVertex(u);
		}
	}
}

bool DStarLite::findPath() {
	cout<<"Search the shortest path with D*Lite."<<endl;
	m_pLast = m_pStart;
	initialize();
	try{
		computeShortestPath();
		while (!isSame(m_pStart, m_pGoal)) {
			//there is no known path
			if (equal(m_pStart->g, INF))
				return false;

			minCPlusG(m_pStart, &m_pStart);

			if (g_changed.size() == 0)
				continue;

			m_km += calcH(m_pLast);
			m_pLast = m_pStart;
			for (int i=0; i<g_changed.size(); i++){
				vertex* s = g_changed[i];
				MazeCell* cell = &m_maze[s->row][s->col];
				if (cell->type == s->type)
					continue;
				cell->copyFrom(*s);
				updateVertex(cell);
			}
			computeShortestPath();
		}
		return true;
	}
	catch (exception & e) {
		cout << "Standard exception: " << e.what() << endl;
	}
	return false;
}
