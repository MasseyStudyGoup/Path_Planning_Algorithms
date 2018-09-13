/*
 * IDAStar.cpp
 *
 *  Created on: 2018Äê9ÔÂ12ÈÕ
 *      Author: ygli
 */
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "DStarLite.h"

#include "IdaStar.h"

IdaStar::IdaStar(int rows, int cols) {
	m_rows = rows;
	m_cols = cols;
	m_maze.resize(rows);
	for (int i = 0; i < rows; i++) {
		m_maze[i].resize(cols);
	}

	m_cutoff = 0;
	m_nextCutOff = m_cutoff;

	m_pStart = nullptr;
	m_pGoal = nullptr;
}

void IdaStar::setStart(int row, int col) {
	m_pStart = &m_maze[row][col];
}

void IdaStar::setGoal(int row, int col) {
	m_pGoal = &m_maze[row][col];
}

double IdaStar::calcH(MazeCell* s) {
	int diffY = abs(m_pGoal->row - s->row);
	int diffX = abs(m_pGoal->col - s->col);
	if (HEURISTIC == MANHATTAN) {
		s->h = fmax(diffY, diffX);
	} else {
		s->h = sqrt(diffY * diffY + diffX * diffX);
	}

	return s->h;
}

void IdaStar::updateH() {
	for (int i = 0; i < m_rows; i++) {
		for (int j = 0; j < m_cols; j++)
			calcH(&m_maze[i][j]);
	}
}

void IdaStar::getNeighbours(MazeCell* u, MazeCell** ptrNeighbours) {
	for (int i = 0; i < DIRECTIONS; i++)
		ptrNeighbours[i] = nullptr;

	int left = 1; // 0 is border
	int right = m_cols - 2; //m_cols-1 is border
	int top = 1;
	int bottom = m_rows - 2;

	for (int i = 0; i < DIRECTIONS; i++) {
		int x = u->col + neighbours[i].x;
		int y = u->row + neighbours[i].y;
		if (x >= left && x <= right && y >= top && y <= bottom
				&& m_maze[y][x].type != T_BLOCKED
				&& m_maze[y][x].visited == false)
			ptrNeighbours[i] = &m_maze[y][x];
	}
}

double IdaStar::cost(MazeCell* u, MazeCell* s) {
	int diffY = abs(u->row - s->row);
	int diffX = abs(u->col - s->col);
	return sqrt(diffY*diffY+diffX*diffX);
}

void IdaStar::initialise() {
	m_path.empty();
	for (int i = 1; i < m_rows - 1; i++) {
		for (int j = 1; j < m_cols - 1; j++) {
			m_maze[i][j].g = INF;
			m_maze[i][j].visited = false;
			calcH(&m_maze[i][j]);
		}
	}

	if (DStarLite::equal(0, m_cutoff)) {
		m_cutoff = cost(m_pStart, m_pGoal);
	} else {
		m_cutoff = m_nextCutOff;
	}
	m_nextCutOff = INF;
	m_pStart->g = 0;
	m_pStart->visited = true;

	IdaStarPath path;
	path.addCell(m_pStart);
	m_path.push(path);
}

void IdaStar::expandPath(IdaStarPath* path) {
	if (path == nullptr)
		return;

	MazeCell* u = path->last();
	MazeCell* succ[DIRECTIONS];
	getNeighbours(u, succ);
	for (int i = 0; i < DIRECTIONS; i++) {
		if (succ[i] == nullptr)
			continue;

		double f = u->g + u->linkCost[i] + succ[i]->h;
		if (f > m_cutoff) {
			if (f < m_nextCutOff) {
				m_nextCutOff = f;
			}
			continue;
		}

		IdaStarPath newPath;
		newPath.copy(path);
		newPath.addCell(succ[i]);
		m_path.push(newPath);

		succ[i]->visited = true;
		succ[i]->g = u->g + u->linkCost[i];
	}
}

IdaStarPath* IdaStar::getPath() {
	if (m_path.size() == 0)
		return nullptr;
	return &m_path.top();
}

bool IdaStar::findPath() {
	cout<<"Search the shortest path with IDA*."<<endl;
	initialise();
	while (true) {
		IdaStarPath path = m_path.top();
		m_path.pop();
		if (path.reaches(m_pGoal))	{
			m_path.empty();
			m_path.push(path);
			cout<<"The shortest path: ";
			for (int i = 0; i < path.cells.size(); i++) {
				cout<<"["<<path.cells[i]->row<<", "<<path.cells[i]->col<<"]";
			}
			cout<<endl;
			return true;
		}

		cout<<"Expand path: ";
		for (int i=0; i<path.cells.size();i++)
			cout<<"["<<path.cells[i]->row<<", "<<path.cells[i]->col<<"]";
		cout<<endl;
		expandPath(&path);


		if (m_path.size() == 0){
			if (DStarLite::equal(m_nextCutOff, INF)){
				return false;
			}
			else{
				initialise();
			}
		}

	}
}

