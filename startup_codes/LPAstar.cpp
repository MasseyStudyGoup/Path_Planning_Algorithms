#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include "LPAstar.h"
#include "DStarLite.h"
#include "gridworld.h"

LpaStar::LpaStar(int rows, int cols) {
	m_rows = rows;
	m_cols = cols;

	m_maze.resize(rows);
	for (int i = 0; i < rows; i++) {
		m_maze[i].resize(cols);
	}

	m_pStart = nullptr;
	m_pGoal = nullptr;
	m_pLast = nullptr;
}

void LpaStar::setStart(int row, int col) {
	m_pStart = &m_maze[row][col];
}

void LpaStar::setGoal(int row, int col) {
	m_pGoal = &m_maze[row][col];
}

void LpaStar::initialise() {
	m_U.empty();
	for (int i = 0; i < m_rows; i++) {
		for (int j = 0; j < m_cols; j++) {
			m_maze[i][j].g = INF;
			m_maze[i][j].rhs = INF;
		}
	}
	m_pStart->rhs = 0;
	calcKey(m_pStart);
	m_U.insert(m_pStart);

}

double LpaStar::calcH(MazeCell* s) {
	int diffY = abs(m_pGoal->row - s->row);
	int diffX = abs(m_pGoal->col - s->col);
	if (HEURISTIC == MANHATTAN) {
		s->h = fmax(diffY, diffX);
	} else { // EUCLIDEAN no need to calculate square root
		s->h = diffY * diffY + diffX * diffX;
	}

	return s->h;
}

void LpaStar::updateH() {
	for (int i = 0; i < m_rows; i++) {
		for (int j = 0; j < m_cols; j++)
			calcH(&m_maze[i][j]);
	}
}

double* LpaStar::calcKey(MazeCell* s) {
	s->key[0] = fmin(s->g, s->rhs + calcH(s));
	s->key[1] = fmin(s->g, s->rhs);
	return s->key;
}

void LpaStar::updateKey() {
	for (int i = 0; i < m_rows; i++) {
		for (int j = 0; j < m_cols; j++) {
			calcKey(&m_maze[i][j]);
		}
	}
}

double LpaStar::minCPlusG(MazeCell* u) {
	MazeCell* succ[DIRECTIONS];
	getNeighbours(u, succ);
	double cg = INF;
	for (int i = 0; i < DIRECTIONS; i++) {
		MazeCell* s = succ[i];
		if (s == nullptr)
			continue;
		if (u->linkCost[i] + s->g < cg) {
			cg = u->linkCost[i] + s->g;
		}
	}

	return cg;
}

void LpaStar::getNeighbours(MazeCell* u, MazeCell** ptrPred) {
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

void LpaStar::updateVertex(MazeCell* u) {
	if (u == nullptr)
		return;

	if (!DStarLite::isSame(u, m_pStart)) {
		MazeCell* result = nullptr;
		u->rhs = minCPlusG(u);
	}

	m_U.remove(u->row, u->col);

	if (!DStarLite::equal(u->g, u->rhs)) {
		calcKey(u);
		m_U.insert(u);
	}
}

void LpaStar::computeShortestPath() {
	MazeCell* Pred[DIRECTIONS];
	while (m_U.size() > 0
			&& (DStarLite::lessThan(m_U.top()->key, calcKey(m_pGoal))
					|| !DStarLite::equal(m_pGoal->rhs, m_pGoal->g))) {
		MazeCell* u = m_U.pop();
		getNeighbours(u, Pred);
		if (u->g > u->rhs) {
			u->g = u->rhs;
			for (int i = 0; i < DIRECTIONS; i++) {
				updateVertex(Pred[i]);
			}
		} else {
			u->g = INF;
			for (int i = 0; i < DIRECTIONS; i++) {
				updateVertex(Pred[i]);
			}
			updateVertex(u);
		}
	}
}

bool LpaStar::findPath() {
	initialise();
	try {
		computeShortestPath();
/*
		if (g_changed.size() == 0)
			continue;
		for (int i = 0; i < g_changed.size(); i++) {
			vertex* s = g_changed[i];
			MazeCell* cell = &m_maze[s->row][s->col];
			if (cell->type == s->type)
				continue;
			cell->copyFrom(*s);
			updateVertex(cell);
		}
		*/
		return true;
	} catch (exception & e) {
		cout << "Standard exception: " << e.what() << endl;
	}

	return false;
}
