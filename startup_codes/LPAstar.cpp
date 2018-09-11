#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include "LPAstar.h"
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

double LpaStar::minValue(double g_, double rhs_) {
	if (g_ <= rhs_) {
		return g_;
	} else {
		return rhs_;
	}
}

int LpaStar::maxValue(int v1, int v2) {

	if (v1 >= v2) {
		return v1;
	} else {
		return v2;
	}
}

double LpaStar::calc_H(int x, int y) {

	int diffY = abs(goal->y - y);
	int diffX = abs(goal->x - x);

	//maze[y][x].h = (double)maxValue(diffY, diffX);
	return (double) maxValue(diffY, diffX);
}

void LpaStar::updateHValues() {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			maze[i][j].h = calc_H(j, i);
		}
	}

	start->h = calc_H(start->x, start->y);
	goal->h = calc_H(goal->x, goal->y);
}

void LpaStar::calcKey(int x, int y) {
	double key1, key2;

	key2 = minValue(maze[y][x].g, maze[y][x].rhs);
	key1 = key2 + maze[y][x].h;
}

void LpaStar::calcKey(LpaStarCell *cell) {
	double key1, key2;

	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + cell->h;

	cell->key[0] = key1;
	cell->key[1] = key2;
}

void LpaStar::updateAllKeyValues() {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			calcKey (&maze[i][j]);
		}
	}

	calcKey (start);
	calcKey (goal);
}

